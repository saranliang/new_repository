#include <math.h>
#include<ctime>
#include<random>
#include"dspr_vtk.h"
#include"myicp_vtk.h"
#include <atomic>
#include"MathTool.h"

const float GOAlERR = 25.0;   //使用const变量替代宏

void DSPR::align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess)
{
    if (isinit == false)      //初始化步骤，这里是初始化input_transformed变量
    {
        target_kdtree->BuildLocatorFromPoints(target_);
        isinit = true;
    }
    //试运行ICP，计算误差大小，判断是否基本贴合
   MyICP  pre_icp;
   pre_icp.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
   pre_icp.setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
   pre_icp.setInputSource(input_);        //source经过变换到target,探针获取的特征点
   pre_icp.setInputTarget(target_);        //target不变，CT模型
   vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
   pre_icp.align(nonsense, guess); //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵==(先应用粗配准)

   float icp_error = pre_icp.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
   Eigen::Matrix4f icp_guess = pre_icp.getFinalTransformation();
   RangeofAngle = icp_error*1.5;     //根据单次ICP的配准情况，设定角度的震荡范围
   //即ICP后的误差越大，其是最优解的概率越小，所需震荡的范围也越大
   std::cout<<"震荡范围是："<<RangeofAngle<<std::endl;
   if(RangeofAngle>180.0)
   {
       RangeofAngle = 180.0;
   }
   RangeofAngle = 10.0;   //度数太小如10.0则不能纠正z轴旋转
   //----------------------------------------------应用当前变换
   //一、直接设定为最大值，即从离散处开始震荡
    float guess_error = 10000.0;
    //二、设定为粗配准结果的误差

    //三、设定为ICP之后的误差标准
    //guess_error = icp_error;
    //-----------------------------------------------
    int p = 10;                      //扰动结果的个数
    //int p = 8;                      //扰动结果的个数
    int OutItrations = 30;           //扰动迭代最大次数

    Eigen::Matrix4f finall_opt_transform;
    std::map<float,Eigen::Matrix4f> mapofomp;
   //累计进行10次DSPR,取其中效果最好的一个
   //并行程序编写时，内部不能有对共享变量的写操作，可以仅有读；这是为了避免发生读写冲突；若使用互斥锁，则会有线程在等待从而造成程序变慢。
   //std::atomic<bool> isStop(false);
   //#pragma omp parallel for //num_threads(4)   //omp要求所有循环都执行
   //for(int j=0;j<10;j++)
   //{
       //int i = j%5;    //从0-4,运行两遍
        //int i = j;
       int i=0;
       printf("---------------------%d，线程号：%d\n", i,omp_get_thread_num());   //omp_get_thread_num()
       Eigen::MatrixXf perturb_trans =  Eigen::MatrixXf::Zero(4,4*p); //初始化p个矩阵，其中矩阵初始化为缺省参数
       int k =0;                                  //当前扰动迭代次数

       Eigen::Matrix4f inside_finall_transform = guess; //初始化震荡中心为粗配准结果
       //Eigen::Matrix4f inside_finall_transform = icp_guess;

       float inside_finall_EMS = 10000.0;    //初始化最佳误差为guess_error
       //float inside_finall_EMS = guess_error

       float rangofangle = (i+1)*RangeofAngle;

       clock_t startTime = clock();
       //while (k<OutItrations&&inside_finall_EMS>25.0)
       while (k<OutItrations)
       {
           //计算扰动结果,在上一轮ICP的finall_transform的基础上扰动
           //float ratio = 10.0*((float)(OutItrations-k))/(float)OutItrations;
           float ratio = (float)(OutItrations-k)/(float)OutItrations;

           //float angle = M_PI*ratio*RangeofAngle/180.0;    //固定震荡幅度
           float angle = M_PI*ratio*rangofangle/180.0;      //震荡幅度逐步增大

           Perturb( perturb_trans,p,angle);   //注意此时的扰动没有施加平移
           //DeterminePerturb(perturb_trans,inside_finall_transform, p,angle);
           //查找所有扰动结果中最优的
          Eigen::Matrix4f opt_perturn =inside_finall_transform ;              //是否要保证每一次的最优震荡结果要优于震荡前的结果？？？
          if(FindOptPerturb( perturb_trans,opt_perturn,inside_finall_transform, guess, inside_finall_EMS)==true)     //用配准效果最好的随机矩阵作为初始矩阵
          {
                inside_finall_transform =opt_perturn;    //最佳配准矩阵
               //printf("第%d次内循环迭代，更新误差为：%f\n", k,inside_finall_EMS);
          }
           //std::cout<<"第"<<k<<"迭代，误差为："<<inside_finall_EMS<<std::endl;
           k++; //迭代次数加1
       }
       clock_t endTime = clock();
       std::cout<<"单次循环时间为："<<(double)(endTime-startTime)/1000.0<<std::endl;

       mapofomp[inside_finall_EMS] = inside_finall_transform;
       printf("第%d次外循环，最小误差为：%f\n", i,inside_finall_EMS);
   //}

   if(mapofomp.begin()->first-icp_error>1.0)
   {
       finall_transform = pre_icp.getFinalTransformation();
       EMS = icp_error;
       std::cout<<"最终选择ICP，误差为"<<EMS<<std::endl;
   }
   else {
       finall_transform = mapofomp.begin()->second;
       EMS = mapofomp.begin()->first;
       std::cout<<"最终选择DSRP,误差为"<<EMS<<std::endl;
   }

   //还原ICP前的矩阵的旋转
    //Eigen::Matrix4f Reduction = SpacetoPlane*(finall_transform*guess.inverse())*HomoInverse(SpacetoPlane);
    //Eigen::Quaternionf temp_Q = RotationMatrix2Quaternion(Reduction.block(0,0,3,3));
    //float z_angle,y_angle,x_angle;
    //toEulerAngle(temp_Q,x_angle,y_angle,z_angle);
    //std::cout<<"最终的zyx角度"<<z_angle*180.0/M_PI<<" "<<y_angle*180.0/M_PI<<" "<<x_angle*180.0/M_PI<<std::endl;
    //std::cout<<"最终的平移量"<<Reduction(0,3)<<" "<<Reduction(1,3)<<" "<<Reduction(2,3)<<std::endl;
}
//Ratio代表逐步减小的震荡幅度,这样的随机性太强了，没有遍历解空间来的稳定
//扰动太小无法跳出局部最小，扰动太大则收敛的可能性太小------最好是绕自身坐标系的相对旋转
//@perturbTrans: 震荡的矩阵合集4*40,这里仅仅是绕中心旋转的震荡矩阵
//@source:       矩阵震荡的基础
//@numPerturb:   震荡的矩阵个数
//@Ratio:        震荡的角度(弧度制PI)
 void DSPR::Perturb( Eigen::MatrixXf& perturbTrans,int numPerturb,float Angle)
 {
     for(int i=0;i<numPerturb;i++)
     {
            Eigen::Matrix4f temp;
            static std::default_random_engine e{ std::random_device{}() };   //random_device根据系统的各种实时参数生成随机值
            std::normal_distribution<float> distribution_r(0.0, Angle);   //(期望，标准差) ，分布取值的概率：期望+—2*标准差 = 95%

            float a,b,c;
            a = distribution_r(e);
            b = distribution_r(e);
            c = distribution_r(e);

            //std::cout<<"震荡范围："<<Angle*180.0/M_PI<<" 震荡角度为"<<a*180.0/M_PI<<"  "<<b*180.0/M_PI<<"    "<<c*180.0/M_PI<<std::endl;

            //因为经历粗配准之后，矩阵在平移误差上已经相差很小了
//            float MaxSizeOfCloud = 2.0;
//            std::normal_distribution<float> distribution_t(0.0,MaxSizeOfCloud*Ratio);   //(均值、方差)
//            float t_1,t_2,t_3;
//            t_1 = distribution_t(e);
//            t_2 = distribution_t(e);
//            t_3 = distribution_t(e);
//           temp = Translation(t_1,t_2,t_3)*RotateZ(c)*RotateY(b)*RotateX(a);

           temp = RotateZ(c)*RotateY(b)*RotateX(a);
           perturbTrans.block(0,i*4,4,4)= temp;              //为第i个震荡矩阵赋值，该矩阵是完整的矩阵变换。可否构建完整的解空间来避免重复尝试------回溯法？
                                                             //本质是遍历整个解空间作为初始矩阵进行ICP
     }
 }
 void DSPR::DeterminePerturb( Eigen::MatrixXf& perturbTrans,int numPerturb,float Angle)
 {
     int x_axis;
     int y_axis;
     int z_axis;
     //分别绕三维坐标系中的8个象限的中心方向旋转Angle度
     for(int j = 0; j < numPerturb; j++)
     {
         x_axis = 1 + (j&1)*(-2);     //每隔1个
         y_axis = 1 + (j>>1&1)*(-2) ; //每隔2个
         z_axis = 1 + (j>>2&1)*(-2) ; //每隔4个
         Eigen::AngleAxisf V1( Angle, Eigen::Vector3f(x_axis,  y_axis, z_axis).normalized());
          Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();

          temp.block(0,0,3,3) = V1.toRotationMatrix();
          perturbTrans.block(0,j*4,4,4)= temp;

          Eigen::Quaternionf temp_Q = RotationMatrix2Quaternion(V1.toRotationMatrix());
          float z_angle,y_angle,x_angle;
          toEulerAngle(temp_Q,x_angle,y_angle,z_angle);
          //std::cout<<"第"<<j<<"次震荡的zyx角度"<<z_angle*180.0/M_PI<<" "<<y_angle*180.0/M_PI<<" "<<x_angle*180.0/M_PI<<std::endl;
     }
 }
//@perturbTran:生成的旋转扰动矩阵
//@optPertur  :最优的扰动矩阵
//@source     :扰动矩阵施加的对象矩阵
//@finallEMS  :最优的扰动矩阵对应的匹配误差
//@guess      :设定的粗配准，用于显示相对其的扰动矩阵
bool DSPR::FindOptPerturb(const Eigen::MatrixXf&  perturbTran,Eigen::Matrix4f& optPerturn,Eigen::Matrix4f souece,const Eigen::Matrix4f& guess,float& finallEMS)
{
    bool flag = false;

    //震荡后先比较再ICP
    float temp_min = 100000000.0;
    Eigen::Matrix4f temp_min_matrix = Eigen::Matrix4f::Identity();
    int choosed_index = -1;    //最终选定的震荡矩阵的下标
    for(int i=0;i<perturbTran.cols()/4;i++)
    {
        //---------------------------------------------应用当前矩阵后的误差计算
        Eigen::Matrix4f temp_matrix = perturbTran.block(0,i*4,4,4);
        temp_matrix = HomoInverse(SpacetoPlane)*temp_matrix*SpacetoPlane;     //将扰动施加在以自己为旋转中心的坐标系中
        temp_matrix = temp_matrix*souece;                                     //将最终的扰动矩阵施加在当前的最优矩阵中

        float guess_error = 0.0;
        vtkSmartPointer<vtkPoints> input_transformed = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
        vtk::transformation<float>(input_, input_transformed, temp_matrix);

        //---------------------------------------------找对应点的方法(初始估计的误差)
        double *testPoint;    //待查找点
        vtkSmartPointer<vtkIdList> index = vtkSmartPointer<vtkIdList>::New();
        for (int i = 0;i < M;i++)
        {
            testPoint = input_->GetPoint(i);
            target_kdtree->FindClosestNPoints(1, testPoint, index);

            vtkIdType point_ind = index->GetId(0);
            double *goal = target_->GetPoint(point_ind);

            float temp_distance = std::sqrt(std::pow((testPoint[0]-goal[0]),2)+std::pow((testPoint[1]-goal[1]),2)+std::pow((testPoint[2]-goal[2]),2));
            guess_error+=temp_distance;
        }

      if(guess_error<temp_min)
      {
          temp_min = guess_error;
          temp_min_matrix = temp_matrix;
          choosed_index = i;
      }
    }

   //作为初始矩阵的ICP后再评价误差
    Eigen::Matrix4f icp_result;
   //ICP组合
    MyICP  icp_1 ;
    icp_1.Para = this->Para;
    icp_1.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
    icp_1.setMaximumIterations(20);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
    icp_1.setInputSource(input_);        //source经过变换到target,探针获取的特征点
    icp_1.setInputTarget(target_);        //target不变，CT模型
    vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
    icp_1.align(nonsense, temp_min_matrix);
    temp_min = icp_1.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
    icp_result = icp_1.getFinalTransformation();

    if(temp_min<finallEMS)
    {
        optPerturn =  icp_result;                                                  //最佳初始矩阵
        flag  = true;
        finallEMS  = temp_min;

//        //还原ICP前的矩阵的旋转
//         Eigen::Matrix4f Reduction = SpacetoPlane*(temp_min_matrix*guess.inverse())*HomoInverse(SpacetoPlane);
//         Eigen::Quaternionf temp_Q = RotationMatrix2Quaternion(Reduction.block(0,0,3,3));
//         float z_angle,y_angle,x_angle;
//         toEulerAngle(temp_Q,x_angle,y_angle,z_angle);
//         std::cout<<"ICP前的zyx角度"<<z_angle*180.0/M_PI<<" "<<y_angle*180.0/M_PI<<" "<<x_angle*180.0/M_PI<<std::endl;
//         std::cout<<"ICP前的平移量"<<Reduction(0,3)<<" "<<Reduction(1,3)<<" "<<Reduction(2,3)<<std::endl;

//         //根据计算出来的结果，判断是否一致
//         Eigen::Matrix4f ola  = RotateZ(z_angle)*RotateY(y_angle)*RotateX(x_angle);
//         std::cout<<"实际的变换矩阵"<<Reduction<<std::endl;
//         std::cout<<"尝试还原的矩阵"<<ola<<std::endl;

//        //还原ICP后的矩阵的旋转
//         Eigen::Matrix4f Reduction__ = SpacetoPlane*(optPerturn*guess.inverse())*HomoInverse(SpacetoPlane);
//         Eigen::Quaternionf temp_Q__ = RotationMatrix2Quaternion(Reduction__.block(0,0,3,3));
//         float z_angle__,y_angle__,x_angle__;
//         toEulerAngle(temp_Q__,x_angle__,y_angle__,z_angle__);
//         std::cout<<"ICP后的zyx角度"<<z_angle__*180.0/M_PI<<" "<<y_angle__*180.0/M_PI<<" "<<x_angle__*180.0/M_PI<<std::endl;
//         std::cout<<"ICP后的平移量"<<Reduction__(0,3)<<" "<<Reduction__(1,3)<<" "<<Reduction__(2,3)<<std::endl;

    }

    return flag;
}

void DSPR::setCenterMatrix(const Eigen::Matrix4f& spacetoPlane)
{
     SpacetoPlane = spacetoPlane;
}
void DSPR::setRangeofAngle(float angle)
{
    RangeofAngle = angle;
}


void DSPR::Analysis()
{
    vtkSmartPointer<vtkPoints> input_transformed = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
    vtk::transformation<float>(input_, input_transformed, finall_transform);

    //---------------------------------------------找对应点的方法(初始估计的误差)
    double testPoint[3];    //待查找点
    double goal[3];
    vtkSmartPointer<vtkIdList> index = vtkSmartPointer<vtkIdList>::New();
    for (int i = 0;i < M;i++)
    {
        input_transformed->GetPoint(i,testPoint);
        target_kdtree->FindClosestNPoints(1, testPoint, index);

        vtkIdType point_ind = index->GetId(0);
        target_->GetPoint(point_ind,goal);

        float temp_distance = std::sqrt(std::pow((testPoint[0]-goal[0]),2)+std::pow((testPoint[1]-goal[1]),2)+std::pow((testPoint[2]-goal[2]),2));
         std::cout << "第" << i << "个点的误差：" <<  temp_distance << std::endl;
    }
}
void DSPR::setPara(std::string RobostName,float p)   //设置矩阵估计时的鲁棒函数
{
   Para = Parameters(RobostName,p);
}



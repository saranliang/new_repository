//#ifndef DQKFICP_H
//#define DQKFICP_H

//#include"dspr_vtk.h"
//#include"myicp_vtk.h"
//#include"MathTool.h"
//#include"dualquat_base.h"
//#include <math.h>
//#include<ctime>
//#include<random>
//#include <atomic>

//class DQKF:public DSPR     //继承传统配准方法
//{
//    public:
//    DQKF(){	}
//    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess); //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵

//    virtual ~DQKF(){}
//protected:
//   virtual void Perturb( Eigen::MatrixXf& perturbTrans,int numPertub);                 //震荡模型，无需规定幅度
//    virtual bool FindOptPerturb(const Eigen::MatrixXf&  perturbTran,Eigen::Matrix4f& optPerturn,Eigen::Matrix4f souece, const Eigen::Matrix4f& guess,float& finallEMS);    //找到最优震荡结果
//private:
//    Eigen::MatrixXf COVR;            //四元数表示下的协方差矩阵，表示当前参数的不确定性，即可用来表示角度的震荡范围

//};
//class PSPR:public ICPBASE
//{
//public:
//    PSPR():
//        correspondence_(new Correspondences)
//        ,correspondence_estimation(new CorrespondenceEstimationMY)  //对应点估计法已经被初始化了
//    {}
//    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess)override;
//    virtual void Analysis()override;
//    Eigen::Matrix4f getcov();
//protected:
//    CorrespondenceEstimationMY* correspondence_estimation;
//    boost::shared_ptr<Correspondences> correspondence_;
//    Eigen::Matrix4f COVR;
//};

////从向量生成矩阵
//Eigen::Matrix3f cross_product(const Eigen::Vector3f a)
//{
//    Eigen::Matrix3f result = Eigen::Matrix3f::Zero();
//    result[0,1] = -1.0*a(2);
//    result[1,0] = 1.0*a(2);
//    result[0,2] = 1.0*a(1);
//    result[2,0] = -1.0*a(1);
//    result[1,2] = -1.0*a(0);
//    result[2,1] = 1.0*a(0);
//    return result;
//}

//void DQKF::align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f &guess)
//{
//    MyICP  pre_icp;
//    pre_icp.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
//    pre_icp.setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
//    pre_icp.setInputSource(input_);        //source经过变换到target,探针获取的特征点
//    pre_icp.setInputTarget(target_);        //target不变，CT模型
//    vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
//    pre_icp.align(nonsense, guess); //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵==(先应用粗配准)

//    float icp_error = pre_icp.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
//    Eigen::Matrix4f icp_guess = pre_icp.getFinalTransformation();

//   int p = 10;                      //扰动结果的个数
//   int OutItrations = 30;           //扰动迭代最大次数
//   int k=0;                         //当前迭代次数
//   float inside_finall_EMS = 10000.0;    //初始化最佳误差为guess_error

//   Eigen::MatrixXf perturb_trans =  Eigen::MatrixXf::Zero(4,4*p); //初始化p个矩阵，其中矩阵初始化为缺省参数
//   Eigen::Matrix4f inside_finall_transform = guess; //初始化震荡中心为粗配准结果

//   Eigen::Matrix4f finall_opt_transform;
//   clock_t startTime = clock();
//   while (k<OutItrations)
//   {
//       Perturb( perturb_trans,p);   //注意此时的扰动没有施加平移


//       //查找所有扰动结果中最优的
//      Eigen::Matrix4f opt_perturn =inside_finall_transform ;              //是否要保证每一次的最优震荡结果要优于震荡前的结果？？？
//      if(FindOptPerturb( perturb_trans,opt_perturn,inside_finall_transform, guess, inside_finall_EMS)==true)     //用配准效果最好的随机矩阵作为初始矩阵
//      {
//            inside_finall_transform =opt_perturn;    //最佳配准矩阵
//           //printf("第%d次内循环迭代，更新误差为：%f\n", k,inside_finall_EMS);
//      }
//       //std::cout<<"第"<<k<<"迭代，误差为："<<inside_finall_EMS<<std::endl;
//       k++; //迭代次数加1
//   }
//   clock_t endTime = clock();

//   printf("最小误差为：%f\n",inside_finall_EMS);

//   if(inside_finall_EMS-icp_error>1.0)
//   {
//       finall_transform = icp_guess;
//       EMS = icp_error;
//       std::cout<<"最终选择ICP，误差为"<<EMS<<std::endl;
//   }
//   else {
//       finall_transform = inside_finall_transform;
//       EMS = inside_finall_EMS;
//       std::cout<<"最终选择DSRP,误差为"<<EMS<<std::endl;
//   }



//}
//void DQKF::Perturb( Eigen::MatrixXf& perturbTrans,int numPertub)
//{
//    for(int i=0;i<numPertub;i++)
//    {
//           Eigen::Matrix4f temp;
//           static std::default_random_engine e{ std::random_device{}() };   //random_device根据系统的各种实时参数生成随机值
//           std::normal_distribution<float> distribution_w(0.0, sqrt(COVR(0,0)));   //(期望，标准差) ，分布取值的概率：期望+—2*标准差 = 95%
//           std::normal_distribution<float> distribution_x(0.0, sqrt(COVR(1,1)));   //(期望，标准差) ，分布取值的概率：期望+—2*标准差 = 95%
//           std::normal_distribution<float> distribution_y(0.0, sqrt(COVR(2,2)));   //(期望，标准差) ，分布取值的概率：期望+—2*标准差 = 95%
//           std::normal_distribution<float> distribution_z(0.0, sqrt(COVR(3,3)));   //(期望，标准差) ，分布取值的概率：期望+—2*标准差 = 95%

//          float w,x,y,z;
//          w = distribution_w(e);
//          x = distribution_x(e);
//          y = distribution_y(e);
//          z = distribution_z(e);

//          Eigen::Quaternionf temp_q(w,x,y,z);
//          Eigen::Matrix4f temp_m = Eigen::Matrix4f::Identity();
//          temp_m.block(0,0,3,3) = temp_q.matrix();

//          perturbTrans.block(0,i*4,4,4)= temp;              //为第i个震荡矩阵赋值，该矩阵是完整的矩阵变换。可否构建完整的解空间来避免重复尝试------回溯法？
//                                                            //本质是遍历整个解空间作为初始矩阵进行ICP
//    }
//}
//bool DQKF::FindOptPerturb(const Eigen::MatrixXf&  perturbTran,Eigen::Matrix4f& optPerturn,Eigen::Matrix4f souece, const Eigen::Matrix4f& guess,float& finallEMS)
//{
//    bool flag = false;

//    //震荡后先比较再ICP
//    float temp_min = 100000000.0;
//    Eigen::Matrix4f temp_min_matrix = Eigen::Matrix4f::Identity();
//    int choosed_index = -1;    //最终选定的震荡矩阵的下标
//    for(int i=0;i<perturbTran.cols()/4;i++)
//    {
//        //---------------------------------------------应用当前矩阵后的误差计算
//        Eigen::Matrix4f temp_matrix = perturbTran.block(0,i*4,4,4);
//        temp_matrix = HomoInverse(SpacetoPlane)*temp_matrix*SpacetoPlane;     //将扰动施加在以自己为旋转中心的坐标系中
//        temp_matrix = temp_matrix*souece;                                     //将最终的扰动矩阵施加在当前的最优矩阵中

//        float guess_error = 0.0;
//        vtkSmartPointer<vtkPoints> input_transformed = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
//        vtk::transformation<float>(input_, input_transformed, temp_matrix);

//        //---------------------------------------------找对应点的方法(初始估计的误差)
//        double *testPoint;    //待查找点
//        vtkSmartPointer<vtkIdList> index = vtkSmartPointer<vtkIdList>::New();
//        for (int i = 0;i < M;i++)
//        {
//            testPoint = input_->GetPoint(i);
//            target_kdtree->FindClosestNPoints(1, testPoint, index);

//            vtkIdType point_ind = index->GetId(0);
//            double *goal = target_->GetPoint(point_ind);

//            float temp_distance = std::sqrt(std::pow((testPoint[0]-goal[0]),2)+std::pow((testPoint[1]-goal[1]),2)+std::pow((testPoint[2]-goal[2]),2));
//            guess_error+=temp_distance;
//        }

//      if(guess_error<temp_min)
//      {
//          temp_min = guess_error;
//          temp_min_matrix = temp_matrix;
//          choosed_index = i;
//      }
//    }

//   //作为初始矩阵的ICP后再评价误差
//    Eigen::Matrix4f icp_result;
//   //ICP组合
//    PSPR  icp_1 ;
//    icp_1.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
//    icp_1.setMaximumIterations(20);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
//    icp_1.setInputSource(input_);        //source经过变换到target,探针获取的特征点
//    icp_1.setInputTarget(target_);        //target不变，CT模型
//    vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
//    icp_1.align(nonsense, temp_min_matrix);
//    temp_min = icp_1.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
//    icp_result = icp_1.getFinalTransformation();

//    if(temp_min<finallEMS)
//    {
//        optPerturn =  icp_result;                                                  //最佳初始矩阵
//        flag  = true;
//        finallEMS  = temp_min;
//    }

//    return flag;
//}
//void PSPR::align(vtkSmartPointer<vtkPoints>nonsense, const Eigen::Matrix4f &guess)
//{
//    Eigen::Matrix3f rotation = guess.block(0,0,3,3);
//    Eigen::Quaternionf guess_rot(rotation);
//    Eigen::Quaternionf guess_tran(0.0,guess(0,3),guess(1,3),guess(2,3));
//    Eigen::Quaternionf guess_dual = (guess_tran*guess_rot);
//    dualquat::DualQuaternionf guess_DQ(guess_rot,Eigen::Quaternionf(guess_dual.w()/2.0,guess_dual.x()/2.0,guess_dual.y()/2.0,guess_dual.z()/2.0));

//    //1、对偶四元数的初始协方差矩阵4*4(一般设置为0矩阵,即不确定性为0)
//    Eigen::Vector4f xk;     //状态向量(旋转矩阵的四元数，xk[0] = 旋转角度)    h = H*xk   h为伪观测值，令其为0
//    Eigen::Vector3f tk;
//    COVR = Eigen::MatrixXf::Identity(4,4)*0.000001;
//    Eigen::MatrixXf COVT = Eigen::MatrixXf::Identity(3,3)*0.000001;
//    Eigen::Matrix4f H;
//    Eigen::Matrix4f K;
//    Eigen::Matrix4f covh;   //伪观测值的协方差矩阵(即不确定性，由变换参数xk、配准点数据a1\b1\a2\b2引入)

//    vtkSmartPointer<vtkPoints> cloud_transformed =  vtkSmartPointer<vtkPoints>::New();
//    vtkSmartPointer<vtkPoints> temp_transformed =  vtkSmartPointer<vtkPoints>::New();
//    Eigen::Matrix4f finall_matrix = guess;
//    cloud_transformed->DeepCopy(input_);
//    int InnerItrations = 50;
//    float sum_error;
//    for(int i=0;i<InnerItrations;i++)
//    {
//        //1、应用变换矩阵
//        input_transformed->Reset(); //分配的空间不变(读取对应的索引位置还是之前的数)，但是看起来像是置空了，插入的话是从头插入
//        vtk::transformation(input_, input_transformed, finall_matrix);          //执行K-1的变换矩阵
//        //2、查找对应点
//        correspondence_estimation->setInputTarget(target_);							           //target 对应match
//        correspondence_estimation->setInputSource(input_transformed);          //input 对应 query
//        correspondence_->clear();
//        correspondence_estimation->determineCorrespondences(*correspondence_,EMS);  //同时查找对应点、计算误差总和

//        double* p1;
//        double* q1;
//        double* p2;
//        double* q2;

//        //3、计算H
//        Eigen::Vector3f sum_c=Eigen::Vector3f::Zero();
//        Eigen::Vector3f sum_b=Eigen::Vector3f::Zero();

//        Eigen::Vector3f c1,b1,c2,b2;   //b为探针点，query  ;c为模型点，match
//        sum_error = 0.0;
//        for(unsigned int  i=0;i<input_transformed->GetNumberOfPoints()-1;i++)
//        {
//            p1 = input_transformed->GetPoint((*correspondence_)[i].index_query);
//            q1 = target_->GetPoint((*correspondence_)[i].index_match);

//            p2 = input_transformed->GetPoint((*correspondence_)[i+1].index_query);
//            q2 = target_->GetPoint((*correspondence_)[i+1].index_match);


//            b1<<p1[0],p1[1],p1[2];
//            c1<<q1[0],q1[1],q1[2];
//            b2<<p2[0],p2[1],p2[2];
//            c2<<q2[0],q2[1],q2[2];

//            H(0,0) = 0.0;
//            H.block(0,1,1,3) = -1.0*(c1-c2-b1+b2).transpose();
//            H.block(1,0,3,1) = c1-c2-b1+b2;
//            H.block(1,1,3,3) = cross_product(c1-c2+b1-b2);

//            sum_c+=c1;    //统计所有点的坐标
//            sum_b+=b1;

//            sum_error += (c1-b1).norm();
//        }
//        sum_c+=c2;
//        sum_b+=b2;
//        sum_error += (c2-b2).norm();
//        //计算covh
//        covh = Eigen::Matrix4f::Identity()*2.0;    //定义观测数据误差矩阵


//        //4、计算K
//        K  = COVR*H.transpose()*(H*COVR*H.transpose()+covh).inverse();
//        //5、更新x和COVR
////       Eigen::Vector4f pre_xk = xk;
////       xk = pre_xk-K*(H*pre_xk);
////       Eigen::Matrix4f pre_cov = COVR;
////       COVR = (Eigen::Matrix4f::Identity()-K*H)*pre_cov;
////       //归一化，保证状态向量四元数是单位的
////       pre_xk = xk;
////       xk = pre_xk/pre_xk.norm();
////       pre_cov = COVR;
////       COVR = COVR/pow(pre_xk.norm(),2);
//       //6、更新tk和COVT
//       Eigen::Quaternionf temp_q(xk);
//       Eigen::Matrix3f totation = temp_q.matrix();
//       sum_c = sum_c/input_transformed->GetNumberOfPoints();
//       sum_b = sum_b/input_transformed->GetNumberOfPoints();
//       tk = sum_c - totation*sum_b;

//       COVT =Eigen::Matrix3f::Zero();   //因为本方法不对平移变量进行震荡，所以设定T的不确定性为0

//       //更新finall_matrix
//       finall_matrix.block(0,0,3,3) = totation;
//       finall_matrix.block(0,3,3,1) = tk;
//    }
//    finall_transform = finall_matrix;
//    EMS = sum_error;
//}
//Eigen::Matrix4f PSPR::getcov()
//{
//    return COVR;
//}
//#endif // DQKFICP_H

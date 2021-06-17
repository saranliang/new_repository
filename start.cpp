#include "start.h"

#include <math.h>
#include<ctime>
#include<sys/time.h>
#include<random>
#include <omp.h>

#include <iostream>
#include <vector>
#include<string>
#include <set>

#include<vtkPolyDataWriter.h>
#include <vtkDecimatePro.h>
#include <vtkAutoInit.h>
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointPicker.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkKdTree.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSphereSource.h>
#include <vtkNamedColors.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2); //这个必须要加，不然会报错
VTK_MODULE_INIT(vtkInteractionStyle);

#include"myicp_vtk.h"
#include"Robost.h"
#include"MathTool.h"
#include"ptp_vtk.h"
#include"dspr_vtk.h"
#include"dqkficp.h"
#include"pso.h"
#include"fitplane.h"


int ModePointSelect;     //选点事件对应的操作函数：1、新增点 2、修改时选中点 3、移动修改点
std::string ModeDeviation;   //设置偏移点的方法
int Tough;               //自定义偏移时的粗配准参考点随机数
int Fine;                //自定义偏移时的精配准参考点随机数
std::string RegistrationOrder_1;  //配准指令左
std::string RegistrationOrder_2;  //配准指令右
std::string FILENAME;    //所在的文件路径前缀
std::string OPENROUTE;

vtkSmartPointer<vtkPoints> cloud_choosed;        //鼠标选中的簇，主要用来显示
vtkSmartPointer<vtkPoints> new_cloud_choosed;    //选中点的cloud备份
vtkSmartPointer<vtkPoints> cloud_target;         //读取pcd的模拟旋转后的点云
vtkSmartPointer<vtkPolyData> stl_target;

std::vector<std::vector<reference>> SelectPoints;    //保留簇关系的容器，用于后序计算
std::vector<reference> OneArea;                      //set内部只保留不重复的元素

////只在当前文件中使用时就可以设定为静态变量
vtkSmartPointer<vtkRenderWindow> RW;
vtkSmartPointer<vtkRenderWindowInteractor> RWInteractor;
void* WindowShow(void* args)
{

    RW->Render();
    RWInteractor->Start();
}

int IsNext;   //表示是否接着进行精配准
bool IsClose;  //标识当前次测试是否结束

 float XCAngle;
 float YCAngle;
 float ZCAngle;

 float XRAngle;
 float YRAngle;
 float ZRAngle;

 float XRDistance;
 float YRDistance;
 float ZRDistance;
 bool IsCenterRotation;

 double SampleRate;

vtkStandardNewMacro(KeyBoardInteractorStyle);
 void KeyBoardInteractorStyle::OnKeyPress()
 {
   // Get the keypress
   vtkRenderWindowInteractor* rwi = this->Interactor;
   std::string key = rwi->GetKeySym();

   // Output the key that was pressed
   std::cout << "Pressed " << key << std::endl;
   //通过鼠标事件开始多线程
   if (key == "space" )  //Return space
   {
       IsNext=1;  //开始进行精配准
       rwi->GetRenderWindow()->Finalize();
       rwi->TerminateApp();
   };
   if (key == "m" )  //Return Enter
   {
       IsNext = 2;  //开始进复合配准
       rwi->GetRenderWindow()->Finalize();
       rwi->TerminateApp();
   }
   if(key == "i")
   {
       IsNext=3;
       rwi->GetRenderWindow()->Finalize();
       rwi->TerminateApp();
   }
   if (key == "Escape" )  //Return Esc
   {      
       IsNext = 0;
       rwi->GetRenderWindow()->Finalize();
       rwi->TerminateApp();
       IsClose = true;           //关闭配准界面

   }
   // Forward events
   vtkInteractorStyleTrackballCamera::OnKeyPress();
 }
RegistrationSatrt::RegistrationSatrt(vtkSmartPointer<vtkPoints> cloud_target)
{
    cloud_model = cloud_target;     //模型点云
    cloud_scan = vtkSmartPointer<vtkPoints>::New();      //模拟旋转后的点云
    cloud_index = vtkSmartPointer<vtkPoints>::New();     //探针选取的点（带噪声）
    cloud_index_tough= vtkSmartPointer<vtkPoints>::New();//选取的点执行粗变换
    cloud_index_translated = vtkSmartPointer<vtkPoints>::New();//选取的点执行精配准变换
    cloud_tough= vtkSmartPointer<vtkPoints>::New();//旋转后的点云执行粗+精变换
    cloud_translated = vtkSmartPointer<vtkPoints>::New();//旋转后的点云执行粗+精变换

    //构建配准树
    target_kdtree = vtkSmartPointer<vtkKdTree>::New();
    target_kdtree->BuildLocatorFromPoints(cloud_model);

}
//计算Target和TraSource之间所有对应点的整体点云误差
void  RegistrationSatrt::CloudError(vtkSmartPointer<vtkPoints> Target, vtkSmartPointer<vtkPoints> TraSource,float& Sum)
{
    float sum=0.0;
    int i ;
    int point_number = TraSource->GetNumberOfPoints();
    double in[3];
    double target[3];
    Eigen::Vector3f vecofpointsource;
     Eigen::Vector3f vecofpointtarget;
    for (i = 0;i < point_number;i++)
    {        
        TraSource->GetPoint(i,in);
        Target->GetPoint(i,target);
        vecofpointsource <<in[0],in[1],in[2];
        vecofpointtarget <<target[0],target[1],target[2];
        float error = (vecofpointsource - vecofpointtarget).norm();
        sum += error;
    }
    sum = sum/SampleRate;
    std::cout << "所有点误差" << sum << std::endl;
    Sum = sum;
}
//分析验证点云的误差情况
float RegistrationSatrt::AnalysisError()
{
    vtkSmartPointer<vtkPoints> construction = vtkSmartPointer<vtkPoints>::New();
    float temp[3];
    for (int seg_order = 1;seg_order < SelectPoints.size();++seg_order)   //按顺序加入点簇
    {
        for (int j=0; j< SelectPoints[seg_order].size()-1; ++j)
        {
//            for(int ohter = j+1;ohter<SelectPoints[seg_order].size();ohter++)
//            {

//                temp[0] = (SelectPoints[seg_order][j].point[0]+SelectPoints[seg_order][ohter].point[0])/2.0;
//                temp[1] = (SelectPoints[seg_order][j].point[1]+SelectPoints[seg_order][ohter].point[1])/2.0;
//                temp[2] = (SelectPoints[seg_order][j].point[2]+SelectPoints[seg_order][ohter].point[2])/2.0;
//                construction->InsertNextPoint(temp);
//            }
            temp[0] = (SelectPoints[seg_order][j].point[0]+SelectPoints[seg_order][j+1].point[0])/2.0;
            temp[1] = (SelectPoints[seg_order][j].point[1]+SelectPoints[seg_order][j+1].point[1])/2.0;
            temp[2] = (SelectPoints[seg_order][j].point[2]+SelectPoints[seg_order][j+1].point[2])/2.0;
            construction->InsertNextPoint(temp);
        }
    }
    float con_error =  CurrentError(construction);

    vtkSmartPointer<vtkPoints> sign = vtkSmartPointer<vtkPoints>::New();
    for(int i=0;i<SelectPoints[0].size();i++)
    {
        sign->InsertNextPoint(SelectPoints[0][i].point);
    }
    float sign_error =  CurrentError(sign);
    //std::cout<<"验证点集的误差"<<con_error<<std::endl;
    // std::cout<<"标记点集的误差"<<sign_error<<std::endl;
}
//根据一群点计算在当前的最优矩阵下的误差，用于验证当前最优矩阵的鲁棒性
//函数内部的静态变量，怎么改变它的值
float RegistrationSatrt::CurrentError(vtkSmartPointer<vtkPoints> Cloud)
{
      float sum_error=0.0;
      //将当前的最优矩阵应用在Cloud中
      vtkSmartPointer<vtkPoints> input_transformed = vtkSmartPointer<vtkPoints>::New();
       vtk::transformation<float>(Cloud, input_transformed, OptMatrix);

      double *testPoint;    //待查找点
      vtkSmartPointer<vtkIdList> index = vtkSmartPointer<vtkIdList>::New();
      int number =input_transformed->GetNumberOfPoints();
      for (int i = 0;i < number;i++)
      {
          testPoint = input_transformed->GetPoint(i);
          target_kdtree->FindClosestNPoints(1, testPoint, index);

          vtkIdType point_ind = index->GetId(0);
          double *goal = cloud_model->GetPoint(point_ind);

          float temp_distance = std::sqrt(std::pow((testPoint[0]-goal[0]),2)+std::pow((testPoint[1]-goal[1]),2)+std::pow((testPoint[2]-goal[2]),2));
          sum_error+=temp_distance;
      }
      return sum_error;
}
bool RegistrationSatrt::cmp(int x, int y) {
    return x > y;
}

//绕髋臼窝平面的中心法线旋转N度
Eigen::Matrix4f RegistrationSatrt::FossaTransformation(float Angle)
{
     vtkSmartPointer<vtkPoints> cloud_plane = vtkSmartPointer<vtkPoints>::New();     //模型点云
    //读取txt中的点到点云
    FILE *txt_point = NULL;
    txt_point = fopen((OPENROUTE+"left_中心拟合.txt").data(), "rb");
    if (!txt_point)
    {
        cout << "打开文件失败" << endl;
    }
    int index;
    float point[3];
    while (!feof(txt_point))                             //这个读取方法会导致多读一次最后一行
    {
        fscanf(txt_point, "%d %f %f %f ", &index, &point[0], &point[1], &point[2]);
        cloud_plane->InsertNextPoint(point);
    }
    fclose(txt_point);    //关闭该文本

    Eigen::Vector3f circle_para;
    Eigen::Matrix4f spacetoplane;
    spacetoplane = FitCircle3D(cloud_plane, circle_para);
    Eigen::Matrix4f rotation_z = RotateZ(abs(180.0-Angle));                                //真实旋转的角度为180-x，因为这里写的Rotation_z是在右手坐标系中的
    //Eigen::Matrix4f rotation_z = RotateX(abs(180.0-Angle));          //绕x轴旋转，测试算法的收敛性能
    Eigen::Matrix4f planetospace = HomoInverse(spacetoplane);
    Eigen::Matrix4f finall_trans = planetospace*rotation_z*spacetoplane;

    FossaCenter = spacetoplane;
    return finall_trans;
}
void  RegistrationSatrt::RegistrationImplement(std::string RegistrationOrder)
{
    //先按行分解读取指令
    std::vector<std::string> vec;
    std::string tmp;
    for (std::string::iterator it=RegistrationOrder.begin(); it != RegistrationOrder.end();it++)
    {
        char c = *it;
        if (c != '\n')
        {
            tmp += c;
        }
        else
        {
            vec.push_back(tmp);
            tmp.clear();
        }
    }
    ModeTrans = vec[0];       //矩阵估计的方法
    ModePara = vec[1];        //权值函数
    ParameterValue = std::atof(vec[2].data());     //权值函数对应的参数
    ModeCorr = vec[3];        //匹配点对的方法

   //建立set_size;
    std::vector<int> set_size;
    for(int i=1;i<SelectPoints.size();i++)
    {
       set_size.push_back(SelectPoints[i].size());
    }
    cloud_index->Reset();
    float opt_error = 10000.0;
    vtkSmartPointer<vtkPoints> Finall = vtkSmartPointer<vtkPoints>::New();    //用来接收align时的变换后点云
    //开始执行配准算法
    for (int seg_order = 1;seg_order < SelectPoints.size();seg_order++)   //按顺序加入点簇
    {
        for (std::vector<reference>::iterator it = SelectPoints.at(seg_order).begin(); it != SelectPoints.at(seg_order).end(); it++)
        {
            cloud_index->InsertNextPoint(it->point);      //累计记录所选点簇
        }
    }
    if(ModeTrans=="DSPR")
    {
       DSPR icp;
       icp.setEuclideanFitnessEpsilon(0.00001);
       icp.setMaximumIterations(50);
       icp.setRangeofAngle(40.0);              //在直接使用随机震荡时，使震荡范围较大，（正负180的概率为95.6%）
       icp.setInputSource(cloud_index);
       icp.setInputTarget(cloud_model);
       icp.setCenterMatrix(FossaCenter);
       icp.align(Finall,ToughMatrix);

       //icp.Analysis();  //输出每个点的误差

       OptMatrix = icp.getFinalTransformation();
       OptError = icp.getFitnessScore(); //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了

    }
    else {
        Eigen::Matrix4f TempTough = ToughMatrix;

        if(ModeTrans=="ICP")
        {
            MyICP* icp =  new MyICP();
            icp->setPara(ModePara,ParameterValue);
            icp->setSetSize(set_size);
            icp->setCorrespondence_estimation(ModeCorr);
            icp->setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
            icp->setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
            icp->setInputSource(cloud_index);        //source经过变换到target,探针获取的特征点
            icp->setInputTarget(cloud_model);        //target不变，CT模型
            icp->align(Finall, TempTough);  //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵==(先应用粗配准)

            //icp->Analysis();  //输出每个点的误差

            OptMatrix = icp->getFinalTransformation();
            OptError = icp->getFitnessScore(); //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
            std::cout<<"最终的误差为："<<OptError<<std::endl;    //经过PTP之后有可能误差反而变大
        }
        if(ModeTrans=="PTP")
        {
            clock_t ICPstatTime = clock();
            //先使用传统ICP更新粗配准矩阵
            MyICP pre_icp;
            pre_icp.setEuclideanFitnessEpsilon(0.00001);
            pre_icp.setMaximumIterations(50);
            pre_icp.setInputSource(cloud_index);
            pre_icp.setInputTarget(cloud_model);
            pre_icp.align(Finall,ToughMatrix);

            clock_t ICPendTime = clock();

            PTPlaneICP* icp = new PTPlaneICP();
            icp->setNormals(stl_target);
            TempTough =pre_icp.getFinalTransformation();
            icp->setPara(ModePara,ParameterValue);
            icp->setSetSize(set_size);
            icp->setCorrespondence_estimation(ModeCorr);
            icp->setEuclideanFitnessEpsilon(0.00001);//误差限，若超过该限制，则视为不收敛
            icp->setMaximumIterations(50);           //最大迭代次数，默认为100，这里设为300时收敛结果也一样
            icp->setInputSource(cloud_index);        //source经过变换到target,探针获取的特征点
            icp->setInputTarget(cloud_model);        //target不变，CT模型
            icp->align(Finall, TempTough);  //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵==(先应用粗配准)

             clock_t PTPendTime = clock();
             std::cout<<"ICP time:"<<(double)(ICPendTime-ICPstatTime)/1000.0<<std::endl;
             std::cout<<"PTP time:"<<(double)(PTPendTime-ICPendTime)/1000.0<<std::endl;

            OptMatrix = icp->getFinalTransformation();
            OptError = icp->getFitnessScore(); //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
            std::cout<<"最终的误差为："<<OptError<<std::endl;    //经过PTP之后有可能误差反而变大
        }
        if(ModeTrans=="PSO")
        {
            PSOICP icp(3,8);
            icp.setEuclideanFitnessEpsilon(0.00001);
            icp.setMaximumIterations(50);
            icp.setRangeofAngle(40.0);              //在直接使用随机震荡时，使震荡范围较大，（正负180的概率为95.6%）
            icp.setInputSource(cloud_index);
            icp.setInputTarget(cloud_model);
            icp.setCenterMatrix(FossaCenter);

            icp.align(Finall,ToughMatrix);

            OptMatrix = icp.getFinalTransformation();
            OptError = icp.getFitnessScore(); //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
        }
    }
}
std::string RegistrationSatrt::DecompositionError(const Eigen::Matrix4f& GoalMatrix)
{
    std::string iterations_cnt;
    // Adding text descriptions in each viewport
    std::stringstream ss;

    test_m = GoalMatrix.block(0, 0, 3, 3);
    expr_error = GoalMatrix.block(0, 3, 3, 1);
    exp_euler = test_m.eulerAngles(2, 1, 0);
    exp_euler[2] = 180.0*exp_euler[2];
    exp_euler[1] = 180.0*exp_euler[1];
    exp_euler[0] = 180.0*exp_euler[0];

    Eigen::Vector3f angle_error = exp_euler - sta_euler;
    Eigen::MatrixXf translation_error = expr_error - std_error;
    ss.str("");
    ss << "rotate error(degree): " << std::endl;
    ss.precision(3);
    ss << "x: " << angle_error[2] << " y: " << angle_error[1] << " z: " << angle_error[0] << std::endl;
    ss << "translation error(mm):" << std::endl;
    ss << "x: " << translation_error(0, 0) << " y: " << translation_error(1, 0) << " z: " << translation_error(2, 0) << std::endl;
    iterations_cnt = ss.str();

    return iterations_cnt;
}
std::string  RegistrationSatrt::ShowSave()
{
    //记录整体点云变换后cloud_translated
    cloud_translated->Reset();    //从第一个开始插入，但内存事实上每释放，读取的话仍然可以读到
    vtk::transformation(cloud_scan, cloud_translated, OptMatrix);

    //分析整体点云的误差
     CloudError(cloud_model,cloud_translated,OptError);
     AnalysisError();

    //记录refrence点转换后cloud_index_translated
    cloud_index_translated->Reset();
    vtk::transformation(cloud_index, cloud_index_translated, OptMatrix);

    std::string iterations_cnt = DecompositionError(OptMatrix);
    return iterations_cnt;
}
//argc表示参数个数，argv表示再命令行中输入的参数个数，以空格为分界符，其中argv0]自动存储为程序名
//测试思路，先自定义一个旋转平移矩阵，将model旋转平移得模拟病人的source,将参考点震荡后旋转平移，得模拟探针的点；再进行配准
int RegistrationSatrt::start()
{
    std::vector<std::vector<reference>> pre_SelectPoints = SelectPoints;    //保留簇关系的容器，用于后序计算
    //文件路径中不能有全为数字的文件名
    //pcl::io::loadPCDFile<pcl::PointXYZ>((FILENAME+"03054_right.pcd").data(), *cloud_model);   //已经在RegistrationStart初始化时给出了cloud_target
    //设定旋转矩阵---------------------------------
    Eigen::Matrix4f TransforMat;
    if(IsCenterRotation)
    {
         TransforMat = FossaTransformation(ZCAngle);         //从Model旋转
    }
    else
    {
        TransforMat = RotateX(M_PI*XRAngle / 180.0)*RotateY(M_PI*YRAngle / 180.0)*RotateZ(M_PI*ZRAngle / 180.0)*Translation(XRDistance, YRDistance,ZRDistance);
         FossaTransformation(10.0);  //还是需要使用，用来计算平面点云的法向量矩阵和大小
    }
    //Eigen::Matrix4f StandardMat = Translation(-20.0, -3.0, -6.0)*RotateZ(M_PI / (-4.0))*RotateY(M_PI / (-3.0))*RotateX(M_PI / (-12.0));//计算结果的金标准
    Eigen::Matrix4f StandardMat = HomoInverse(TransforMat);  //计算结果的金标准

    //为整体点云施加旋转
    cloud_translated->Reset();    //从第一个开始插入，但内存事实上每释放，读取的话仍然可以读到
    vtk::transformation(cloud_model, cloud_scan, TransforMat);
    savaPointsVTK(cloud_scan,(FILENAME+"hip_left.vtk").data());

    test_m = StandardMat.block(0, 0, 3, 3);
    sta_euler = test_m.eulerAngles(2, 1, 0);          //表示旋转的顺序是z、x、y,是基于右手准则的右乘法,旋转四元数。
    sta_euler = test_m.eulerAngles(2, 1, 0);
    sta_euler[2] = 180.0*sta_euler[2];
    sta_euler[1] = 180.0*sta_euler[1];
    sta_euler[0] = 180.0*sta_euler[0];
    std_error = StandardMat.block(0, 3, 3, 1);


    //已经自己通过交互选点-----------------------------------------------------------------------------------------------------------

    //获取给定的参考点--------------------------------------------------------------------------------------------------
    std::vector<reference>::iterator it;
    Eigen::MatrixXf  correspond_y(3, SelectPoints.at(0).size());          //这里in是模型的点云：y,  out是扫描的点云：x
    Eigen::MatrixXf  correspond_x(3, SelectPoints.at(0).size());
    vtkSmartPointer<vtkPoints> cloud_Reference = vtkSmartPointer<vtkPoints>::New();//震荡后的参考点+特征点

    //存储旋转震荡前的参考点和特征点
    for (unsigned long index = 0;index < SelectPoints.size();index++)
    {
        std::vector<reference>::iterator it;
        for (it = SelectPoints.at(index).begin();it != SelectPoints.at(index).end();it++)
        {
            cloud_Reference->InsertNextPoint(it->point);           
        }
    }
    savaPointsVTK(cloud_Reference,(FILENAME+"InitialReferenced.vtk").data());
    //为选择的参考点和特征点施加旋转
    for (unsigned long index = 0;index < SelectPoints.size();index++)
    {
        std::vector<reference>::iterator it;
        for (it = SelectPoints.at(index).begin();it != SelectPoints.at(index).end();it++)
        {
            Eigen::MatrixXf temp_point = Eigen::MatrixXf::Ones(4,1);
            temp_point.block(0,0,3,1) <<it->point[0],it->point[1],it->point[2];
            temp_point = TransforMat*temp_point;
            it->point[0] = temp_point(0,0);
            it->point[1] = temp_point(1,0);
            it->point[2] = temp_point(2,0);
        }
    }
    //上次测试震荡后的标志点和参考点，是否这次继续使用:ModeDeviation=="keep"  --不改变，ModeDeviation=="random"----随机生成  ，ModeDeviation=="set"  指定随机种子数
    if (ModeDeviation!="keep")
    {
        //在模型表面震荡,表面每个点之间的距离大概为0.9mm(这里保证了点一定在表面，而不是随机震荡)
        //创建一个KD树,KdTreeFLANN是继承Kdtree的具有三维空间检索能力的子类 ,注意这里仅仅声明了一个对象，而不是共享指针
         vtkSmartPointer<vtkKdTree>  tree_shock = vtkSmartPointer<vtkKdTree>::New();
         tree_shock->BuildLocatorFromPoints(cloud_scan);

         unsigned long k = 10000;
         int j = 0;
         std::vector<float> pointDistance(k);
         vtkSmartPointer<vtkIdList> pointID = vtkSmartPointer<vtkIdList>::New();
         double test_point[3];
        for (it = SelectPoints.at(0).begin(); it != SelectPoints.at(0).end(); it++)
        {
            //添加随机量作为噪声
            std::copy(it->point,it->point+3,test_point);
            tree_shock->FindClosestNPoints(k, test_point, pointID); //这里存储的距离是平方距离
            //添加随机量作为噪声     --高斯分布
            static std::default_random_engine e;
            if(ModeDeviation=="random")
            {
                e = std::default_random_engine{std::random_device{}()};   //random_device根据系统的各种实时参数生成随机值
            }
            else
            {
                e = std::default_random_engine{((unsigned int)(Tough))};  //以固定数作为随机种子数
            }
            static std::uniform_real_distribution<double> u;               //平均分布
            double random_0 = u(e, decltype(u)::param_type(8.0, 10.0));     //(min,max)可以指定取值范围

            //高斯分布
            // std::normal_distribution<double> distribution(0.0, 1.0);   //(均值、方差)
            //distribution(e);


            double far_point[3];
            cloud_scan->GetPoint(pointID->GetId(k-1),far_point);
            float max_distance = std::sqrt(std::pow((test_point[0]-far_point[0]),2)+std::pow((test_point[1]-far_point[1]),2)+std::pow((test_point[2]-far_point[2]),2));
            //std::cout << max_distance << endl;   //输出10000个点中最远点的距离

            unsigned long i;
            double goal[3];
            for (i = 0;i < k;i++)
            {
                vtkIdType point_ind = pointID->GetId(i);
                cloud_scan->GetPoint(point_ind,goal);
                float distance = std::sqrt(std::pow((test_point[0]-goal[0]),2)+std::pow((test_point[1]-goal[1]),2)+std::pow((test_point[2]-goal[2]),2));
                if (std::abs(distance - random_0) < 0.8)
                {
                    //cout << "当前震荡值为" << std::sqrt(pointDistance[i]) << endl;
                   // std::cout << "实际震荡值为" << distance << endl;
                    std::copy(goal,goal+3,it->point);
                    break;
                }
            }
            if (i == k)
            {
                std::cout << "没找到合适的震荡点" << std::endl;
            }
            double target[3];
            cloud_model->GetPoint(it->index,target);
            correspond_y.block(0, j, 3, 1)<<target[0],target[1],target[2];       //将点云中的点转换为矩阵中的一列
            correspond_x.block(0, j, 3, 1)<<it->point[0],it->point[1],it->point[2];      																																																															   //std::cout << "目标：" << cloud_out->points[CP[i]] << endl;
            j++;
        }
        //给特征点选加微量震荡，震荡范围为球形，主要来源是分割的误差，要保证距离为2，而不是每个轴向的位移为2
        for (int i = 1;i < SelectPoints.size();i++)
        {
            for (it = SelectPoints.at(i).begin(); it != SelectPoints.at(i).end(); it++)
            {
                float dinstance = 3.0;
               float temp[3];
                while (dinstance >= 2.0)
                {
                    //添加随机量作为噪声
                    static std::default_random_engine e;
                    if(ModeDeviation=="random")
                    {
                        e = std::default_random_engine{std::random_device{}()};   //random_device根据系统的各种实时参数生成随机值
                    }
                    else
                    {
                        e = std::default_random_engine{((unsigned int)(Fine))};  //以固定数作为随机种子数
                    }
                    static std::uniform_real_distribution<double> u;               //平均分布
                    double random_0 = u(e, decltype(u)::param_type(-2.0, 2.0));     //(min,max)可以指定取值范围
                    double random_1 = u(e, decltype(u)::param_type(-2.0, 2.0));
                    double random_2 = u(e, decltype(u)::param_type(-2.0, 2.0));
                    std::copy(it->point,it->point+3,temp);

                    temp[0] += random_0;
                    temp[1] += random_1;
                    temp[2] += random_2;
                    dinstance = std::sqrt(std::pow((temp[0]-it->point[0]),2)+std::pow(( temp[1]-it->point[1]),2)+std::pow(( temp[2]-it->point[2]),2));
                }
                std::copy(temp,temp+3,it->point);  //it->point = temp
               // std::cout << "距离为：" << dinstance << endl;   //输出误差项
            }
        }
        //存储旋转震荡后的粗配准点和参考点
        //int i = 0;
        cloud_Reference->Reset();
        for (int index = 0;index < SelectPoints.size();index++)
        {
            for (it = SelectPoints.at(index).begin();it != SelectPoints.at(index).end();it++)
            {
                cloud_Reference->InsertNextPoint(it->point);
            }
        }
        savaPointsVTK(cloud_Reference,(FILENAME+"Referenced.vtk").data());
    }
    else
    {
        //读取存贮的震荡后的参考点和特征点
        vtkSmartPointer<vtkPoints> cloud_Reference = vtkSmartPointer<vtkPoints>::New();
        loadPointsVTK(cloud_Reference,(FILENAME+"Referenced.vtk").data());


        int i = 0;
        double y[3];
        double x[3];
        for (it = SelectPoints.at(0).begin();it != SelectPoints.at(0).end();it++)
        {
            cloud_model->GetPoint(it->index,y);               //index是用来标记对应点的
            cloud_Reference->GetPoint(i,x);
            correspond_y.block(0, i, 3, 1) <<y[0],y[1],y[2];   //  模型中对应的参考点
            correspond_x.block(0, i, 3, 1) <<x[0],x[1],x[2];       //	存储的震荡后的参考点																																																															   //std::cout << "目标：" << cloud_out->points[CP[i]] << endl;
            i++;
        }
        for (int index = 1;index < SelectPoints.size();index++)
        {
            for (it = SelectPoints.at(index).begin();it != SelectPoints.at(index).end();it++)
            {
                cloud_Reference->GetPoint(i,x);
                std::copy(x,x+3,it->point);
                i++;
            }
        }
    }
    //粗配准计算-------------------------------------------------------------------------------------------------------------------
     //中心化
     Eigen::MatrixXf meanVec_y = correspond_y.rowwise().mean();   //按照行求均值,得到的是一个列向量,3*1
     Eigen::MatrixXf zeroMeanMat_y = correspond_y;
     for (int i = 0;i < zeroMeanMat_y.cols();i++)      //每一列减去均值
     {
        zeroMeanMat_y.block(0, i, 3, 1) -= meanVec_y;
     }
     //std::cout << zeroMeanMat_in<<endl;
     Eigen::MatrixXf meanVec_x = correspond_x.rowwise().mean();   //按照行求均值,得到的是一个列向量,3*1
     Eigen::MatrixXf zeroMeanMat_x = correspond_x;
     for (int i = 0;i < zeroMeanMat_x.cols();i++)      //每一列减去均值
     {
        zeroMeanMat_x.block(0, i, 3, 1) -= meanVec_x;
     }

     //计算协方差，行数为数据个数，列数为数据维数 这样的数据计算时是 A.T*A
     //但是在矩阵运算时，点的存储形式往往是3*1,所以是A*A.T
     Eigen::MatrixXf covMat = zeroMeanMat_x*zeroMeanMat_y.adjoint();

     //SVD分解求出旋转和平移矩阵，结果的事 y = R*x +T ，即在构建数据时做的变换的逆变换 = 金标准
     Eigen::JacobiSVD<Eigen::MatrixXf> svd(covMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
     Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
     float det = (U*V.transpose()).determinant();  //求行列式
     Eigen::Matrix3f M;
     M << 1, 0, 0, 0, 1, 0, 0, 0, det;
     Eigen::Matrix3f R = V*M*U.transpose();
     Eigen::MatrixXf T = meanVec_y - R*meanVec_x;
     //总结粗变换
     /* 提示: Eigen中的变换矩阵工作原理 :
     |-------> 变换矩阵列
     | 1 0 0 x |  \
     | 0 1 0 y |   }-> 左边是一个3阶的单位阵(无旋转)，这里的定义是非齐次旋转矩阵
     | 0 0 1 z |  /    右边是x,y,z顺序的平移矩阵 ，就是只是将个旋转平移的信息储存
     | 0 0 0 1 |    -> 这一行用不到 (这一行保持 0,0,0,1)
     */
     ////Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
     Eigen::Matrix4f transform_2;
     transform_2.setZero();             //初始化为0矩阵
     transform_2.block(0, 0, 3, 3) = R;
     transform_2.block(0, 3, 3, 1) = T;
     transform_2(3, 3) = 1;
     std::cout << "粗匹配：" << transform_2.matrix() << endl;

    //为粗配准结果矩阵赋值
     if(IsCenterRotation)
     {
         ToughMatrix = Eigen::Matrix4f::Identity();  //若是设定为绕中心旋转，则粗配准设置为单位矩阵
     }
     else
     {
          ToughMatrix = transform_2.matrix();        //若是设定绕世界坐标系旋转，则应用粗配准结果
     }


     //下采样，本来应该在读取stl是就下采样的(同时更新，cloud_model、cloud_scan、stl_target)
     if(SampleRate!=1.0)
     {
         vtkSmartPointer<vtkDecimatePro> decimate =
                 vtkSmartPointer<vtkDecimatePro>::New();
         decimate->SetInputData(stl_target);
         decimate->PreserveTopologyOff();
         decimate->SplittingOn();
         decimate->BoundaryVertexDeletionOn();
         decimate->SetTargetReduction(SampleRate);

         decimate->Update();

         vtkSmartPointer<vtkPolyData> decimated =
                 vtkSmartPointer<vtkPolyData>::New();
         decimated->ShallowCopy(decimate->GetOutput());   //浅拷贝，公用同一份数据

         stl_target = decimated;                          //将指针指向新降采样后的点
         cloud_model->Reset();
         cloud_model->DeepCopy(stl_target->GetPoints());  //更新cloud_model指针指向
         cloud_scan->Reset();
         vtk::transformation(cloud_model, cloud_scan, TransforMat);  //更新cloud_scan指向
     }


    //进行ICP
     cloud_index->Reset();
     for (int seg_order = 1;seg_order < SelectPoints.size();seg_order++)   //按顺序加入点簇
     {
         for (std::vector<reference>::iterator it = SelectPoints.at(seg_order).begin(); it != SelectPoints.at(seg_order).end(); it++)
         {
             cloud_index->InsertNextPoint(it->point);      //累计记录所选点簇
         }
     }

     MyICP* icp =  new MyICP();
     icp->setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
     icp->setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
     icp->setInputSource(cloud_index);        //source经过变换到target,探针获取的特征点
     icp->setInputTarget(cloud_model);        //target不变，CT模型
     vtkSmartPointer<vtkPoints> Finall = vtkSmartPointer<vtkPoints>::New();
     icp->align(Finall, ToughMatrix);  //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵==(先应用粗配准)


     OptMatrix = icp->getFinalTransformation();
     OptError = icp->getFitnessScore(); //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
     std::cout<<"最终的误差为："<<OptError<<std::endl;    //经过PTP之后有可能误差反而变大
     ShowSave(); //更新点云_translated ,并分析取得iterations_cnt


    // 可视化
     vtkSmartPointer<vtkPolyData>  target= vtkSmartPointer<vtkPolyData>::New();
     target->SetPoints(cloud_model);          //设置点坐标
     vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType pid[1];
     for(vtkIdType i=0;i< target->GetPoints()->GetNumberOfPoints();i++)
     {
             pid[0] = i;
            vertices->InsertNextCell(1,pid);
     }
     target->SetVerts(vertices);

     vtk::transformation(cloud_scan,cloud_tough,ToughMatrix);
     vtkSmartPointer<vtkPolyData>  tough_source= vtkSmartPointer<vtkPolyData>::New();
     tough_source->SetPoints(cloud_tough);          //设置点坐标
     tough_source->SetVerts(vertices);

      vtkSmartPointer<vtkPolyData>  icp_source= vtkSmartPointer<vtkPolyData>::New();
      icp_source->SetPoints(cloud_translated);          //设置点坐标
      icp_source->SetVerts(vertices);

     //mapper
     vtkSmartPointer<vtkPolyDataMapper> mapper_target = vtkSmartPointer<vtkPolyDataMapper>::New();
     mapper_target->SetInputData(target);

      vtkSmartPointer<vtkPolyDataMapper> mapper_tough = vtkSmartPointer<vtkPolyDataMapper>::New();
     mapper_tough->SetInputData(tough_source);

     vtkSmartPointer<vtkPolyDataMapper> mapper_icp = vtkSmartPointer<vtkPolyDataMapper>::New();
     mapper_icp->SetInputData(icp_source);
    //actor
      vtkSmartPointer<vtkActor> actor_target = vtkSmartPointer<vtkActor>::New();
     actor_target->SetMapper(mapper_target);
     actor_target->GetProperty()->SetColor(1.0, 1.0, 1.0); //白色  原始target

      vtkSmartPointer<vtkActor> actor_tough = vtkSmartPointer<vtkActor>::New();
     actor_tough->SetMapper(mapper_tough);
     actor_tough->GetProperty()->SetColor(0, 1.0, 0);     //绿色   粗配准结果

     vtkSmartPointer<vtkActor> actor_icp = vtkSmartPointer<vtkActor>::New();
    actor_icp->SetMapper(mapper_icp);
    actor_icp->GetProperty()->SetColor(1.0, 0, 0);        //红色   ICP结果


    //建立两个视图
    vtkSmartPointer<vtkRenderer>  renderer_1 = vtkSmartPointer<vtkRenderer>::New();
     renderer_1->AddActor(actor_target);
       renderer_1->AddActor(actor_tough);
     renderer_1->SetViewport(0,0,0.5,1);

    vtkSmartPointer<vtkRenderer> renderer_2 = vtkSmartPointer<vtkRenderer>::New();
     renderer_2->AddActor(actor_target);
     renderer_2->AddActor(actor_icp);
     renderer_2->SetViewport(0.5,0,1,1);

     //基于全局变量的线程
     RW = vtkSmartPointer<vtkRenderWindow>::New();
     RW->AddRenderer(renderer_1);
     RW->AddRenderer(renderer_2);

     RW->SetSize(1280, 1024);
     RW->SetWindowName("Tough");
    std::string iterations_cnt = DecompositionError(ToughMatrix);   //分析粗配准矩阵的误差

    // Register keyboard callback :用于监控是否按下空格从而开始精配准
    RWInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    RWInteractor->SetRenderWindow(RW);

    vtkSmartPointer<KeyBoardInteractorStyle> style =  vtkSmartPointer<KeyBoardInteractorStyle>::New();
    RWInteractor->SetInteractorStyle(style);

    RW->Render();
    RWInteractor->Start();

    std::cout<<"退出时模式为"<<IsNext<<std::endl;

    //创建线程进行绘制，不能利用同一全局变量在不同时刻对应的不同数据进行两个线程下的绘制，这样数据会冲突。因此只能创建一个线程
//    pthread_t tid;
//    int ret;
//    ret = pthread_create(&tid,NULL,WindowShow,NULL);

     //基于自定义参数的线程
//     cb_args.RW = vtkSmartPointer<vtkRenderWindow>::New();
//     cb_args.RW->AddRenderer(renderer_1);
//     cb_args.RW->AddRenderer(renderer_2);

//     cb_args.RW->SetSize(1280, 1024);
//    std::string iterations_cnt = DecompositionError(ToughMatrix);   //分析粗配准矩阵的误差

//    // Register keyboard callback :用于监控是否按下空格从而开始精配准
//    cb_args.RWInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    cb_args.RWInteractor->SetRenderWindow(cb_args.RW);

//    vtkSmartPointer<KeyBoardInteractorStyle> style =  vtkSmartPointer<KeyBoardInteractorStyle>::New();
//     cb_args.RWInteractor->SetInteractorStyle(style);

//    //创建线程进行绘制，不能利用同一全局变量在不同时刻对应的不同数据进行两个线程下的绘制，这样数据会冲突。因此只能创建一个线程
//    pthread_t tid;
//    int ret;
//    ret = pthread_create(&tid,NULL,WindowShow,&cb_args);
    //创建进程可以直接继续，但是会复制一份当前工作空间，花费较大
//    pid_t ret=fork();
//    if(ret>0)    //执行父进程代码
//    if(ret==0)   //执行子进程代码


    //需要迭代的次数
    vtkSmartPointer<vtkPoints> cloud_index_left = vtkSmartPointer<vtkPoints>::New();     //探针选取的点（带噪声）
    vtkSmartPointer<vtkPoints> cloud_index_right = vtkSmartPointer<vtkPoints>::New();     //探针选取的点（带噪声）

    Eigen::Matrix4f FinalMatrix_Right;   //为了使接口通用，模拟Eigen中的Affine3f
    while (IsClose==false)
    {
        // The user pressed "space" :   开始精配准

        if (IsNext==1)
        {
             vtkSmartPointer<vtkPoints> left_points =  vtkSmartPointer<vtkPoints>::New();
             vtkSmartPointer<vtkPolyData>  left_source= vtkSmartPointer<vtkPolyData>::New();
             vtkSmartPointer<vtkPoints> right_points =  vtkSmartPointer<vtkPoints>::New();
             vtkSmartPointer<vtkPolyData>  right_source= vtkSmartPointer<vtkPolyData>::New();
            //左边视图->
            //尝试添加线程

            {
                RegistrationImplement(RegistrationOrder_1); //根据配准指令执行对应的配准方法
                iterations_cnt = ShowSave(); //更新点云_translated ,并分析取得iterations_cnt

                savaPointsVTK(cloud_translated,(FILENAME+"配准结果_v1.vtk").data());
                savaPointsVTK(cloud_index_translated,(FILENAME+"touched_v1.vtk").data());

                //构建待绘制点云的PolyData
                left_points->DeepCopy(cloud_translated);      //要用深拷贝，因为cloud_translated会变化
                left_source->SetPoints(left_points);          //设置点坐标
                left_source->SetVerts(vertices);
            }
            //右边视图->
            {
                //读取第二条配准指令
                RegistrationImplement(RegistrationOrder_2);
                iterations_cnt = ShowSave(); //更新点云_translated ,并分析取得iterations_cnt
                savaPointsVTK(cloud_translated,(FILENAME+"配准结果_v2.vtk").data());
                savaPointsVTK(cloud_index_translated,(FILENAME+"touched_v2.vtk").data());

                right_points->DeepCopy(cloud_translated);      //要用深拷贝，因为cloud_translated会变化
                right_source->SetPoints(right_points);          //设置点坐标
                right_source->SetVerts(vertices);
            }

             //使用stl显示
             //left_source = stl_target;
             //actor_left->SetUserTransform(trans);  //默认是左乘PreMultiply，显式设定为PostMultiply


            //mapper
            vtkSmartPointer<vtkPolyDataMapper> mapper_left = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper_left->SetInputData(left_source);

             vtkSmartPointer<vtkPolyDataMapper> mapper_right= vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper_right->SetInputData(right_source);
           //actor
             vtkSmartPointer<vtkActor> actor_left = vtkSmartPointer<vtkActor>::New();
            actor_left->SetMapper(mapper_left);
            actor_left->GetProperty()->SetColor(1.0, 0, 0); //白色  原始target

             vtkSmartPointer<vtkActor> actor_right = vtkSmartPointer<vtkActor>::New();
            actor_right->SetMapper(mapper_right);
            actor_right->GetProperty()->SetColor(1.0, 0, 0);     //绿色   粗配准结果 （1.0，0，0）红色

            //render
            renderer_1->RemoveActor(renderer_1->GetActors()->GetLastActor()); //删除掉tough的点云
            renderer_2->RemoveActor(renderer_2->GetActors()->GetLastActor());

            renderer_1->AddActor(actor_left);
            renderer_2->AddActor(actor_right);

           RW->SetWindowName("ICP PTP");
           RW->SetWindowName("ICP PTP");

           RW->Render();
           RWInteractor->Start();

        }
        //为了防止随机性的干扰，将复合配准放在同一个流程中比较：先显示纯DSPR,再显示基于初始矩阵的PTP
        if(IsNext==2)
        {
            std::cout<<"Registration......"<<std::endl;

            std::clock_t DSPRstart = clock();//计时开始
            //struct  timeval   tv_begin,tv_end;
            //gettimeofday(&tv_begin,NULL);


            cloud_index->Reset();
            float opt_error = 10000.0;
            vtkSmartPointer<vtkPoints> Finall = vtkSmartPointer<vtkPoints>::New();    //用来接收align时的变换后点云
            //开始执行配准算法
            for (int seg_order = 1;seg_order < SelectPoints.size();seg_order++)   //按顺序加入点簇
            {
                for (std::vector<reference>::iterator it = SelectPoints.at(seg_order).begin(); it != SelectPoints.at(seg_order).end(); it++)
                {
                    cloud_index->InsertNextPoint(it->point);      //累计记录所选点簇
                }
            }

           //DQKF icp;
           DSPR icp;
           icp.setEuclideanFitnessEpsilon(0.00001);
           icp.setMaximumIterations(50);
           icp.setInputSource(cloud_index);
           icp.setInputTarget(cloud_model);
           icp.setCenterMatrix(FossaCenter);
           icp.align(Finall,ToughMatrix);

           OptMatrix = icp.getFinalTransformation();
           OptError = icp.getFitnessScore();
          std::clock_t  DSPRend = clock();

           iterations_cnt = ShowSave(); //更新点云_translated ,并分析取得iterations_cnt
//           //存储点云
//          savaPointsVTK(cloud_translated,(FILENAME+"配准结果_v1.vtk").data());
//           savaPointsVTK(cloud_index_translated,(FILENAME+"touched_v1.vtk").data());

           //构建待绘制点云的PolyData
           vtkSmartPointer<vtkPolyData>  left_source= vtkSmartPointer<vtkPolyData>::New();
           vtkSmartPointer<vtkPoints> left_points = vtkSmartPointer<vtkPoints>::New();
           left_points->DeepCopy(cloud_translated);
           left_source->SetPoints(left_points);          //设置点坐标
           left_source->SetVerts(vertices);

           //建立set_size;
            std::vector<int> set_size;
            for(int i=1;i<SelectPoints.size();i++)
            {
               set_size.push_back(SelectPoints[i].size());
            }



           PTPlaneICP icp_finall;
           icp_finall.setNormals(stl_target);

           std::clock_t PTPstart = clock();
           //MyICP icp_finall;
           icp_finall.setSetSize(set_size);
           icp_finall.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
           icp_finall.setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
           icp_finall.setInputSource(cloud_index);        //source经过变换到target,探针获取的特征点
           icp_finall.setInputTarget(cloud_model);        //target不变，CT模型           
           icp_finall.align(Finall, OptMatrix);  //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵==(先应用粗配准)
           //icp_finall.Analysis();
           OptMatrix = icp_finall.getFinalTransformation();
           OptError = icp_finall.getFitnessScore(); //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了

           std::clock_t PTPend = clock();//计时结束
           //gettimeofday(&tv_end,NULL);
           std::cout << "The run time is:" <<(double)((DSPRend-DSPRstart))/1000.0<<"+"<<(double)((PTPend - PTPstart))/1000.0 << "ms" << endl;

           std::cout<<"最终的误差为："<<OptError<<std::endl;    //经过PTP之后有可能误差反而变大


           iterations_cnt = ShowSave(); //更新点云_translated ,并分析取得iterations_cnt
           savaPointsVTK(cloud_translated,(FILENAME+"配准结果_v2.vtk").data());
           savaPointsVTK(cloud_index_translated,(FILENAME+"touched_v2.vtk").data());

           //构建待绘制点云的PolyData
           vtkSmartPointer<vtkPolyData>  right_source= vtkSmartPointer<vtkPolyData>::New();
           vtkSmartPointer<vtkPoints> right_points = vtkSmartPointer<vtkPoints>::New();
           right_points->DeepCopy(cloud_translated);
           right_source->SetPoints( right_points);          //设置点坐标
           right_source->SetVerts(vertices);

           //mapper
           vtkSmartPointer<vtkPolyDataMapper> mapper_left = vtkSmartPointer<vtkPolyDataMapper>::New();
           mapper_left->SetInputData(left_source);

            vtkSmartPointer<vtkPolyDataMapper> mapper_right= vtkSmartPointer<vtkPolyDataMapper>::New();
           mapper_right->SetInputData(right_source);
          //actor
            vtkSmartPointer<vtkActor> actor_left = vtkSmartPointer<vtkActor>::New();
           actor_left->SetMapper(mapper_left);
           actor_left->GetProperty()->SetColor(1.0, 0, 0); //白色  原始target

            vtkSmartPointer<vtkActor> actor_right = vtkSmartPointer<vtkActor>::New();
           actor_right->SetMapper(mapper_right);
           actor_right->GetProperty()->SetColor(1.0, 0, 0);     //绿色   粗配准结果 （1.0，0，0）红色

           //render
           renderer_1->RemoveActor(renderer_1->GetActors()->GetLastActor()); //删除掉tough的点云
           renderer_2->RemoveActor(renderer_2->GetActors()->GetLastActor());

           renderer_1->AddActor(actor_left);
           renderer_2->AddActor(actor_right);


           RW->SetWindowName("DSPR PTP");
           RW->SetWindowName("DSPR PTP");

           RW->Render();
           RWInteractor->Start();

        }
        //增量式的加入点簇,并将上一次的搜索结果作为下一次的配准的起点
        if(IsNext==3)
        {
            cloud_index->Reset();
            for (int seg_order = 1;seg_order < SelectPoints.size();seg_order++)   //按顺序加入点簇
            {
                for (std::vector<reference>::iterator it = SelectPoints.at(seg_order).begin(); it != SelectPoints.at(seg_order).end(); it++)
                {
                    cloud_index->InsertNextPoint(it->point);      //累计记录所选点簇
                }
            }
            {
                DSPR pre_icp;
                pre_icp.setPara(ModePara,ParameterValue);
                pre_icp.setEuclideanFitnessEpsilon(0.00001);
                pre_icp.setMaximumIterations(50);
                pre_icp.setInputSource(cloud_index);
                pre_icp.setInputTarget(cloud_model);
                pre_icp.setCenterMatrix(FossaCenter);
                pre_icp.align(Finall,ToughMatrix);

                OptMatrix = pre_icp.getFinalTransformation();
                OptError = pre_icp.getFitnessScore();
            }

            iterations_cnt = ShowSave(); //更新点云_translated ,并分析取得iterations_cnt



            //构建待绘制点云的PolyData
            vtkSmartPointer<vtkPolyData>  left_source= vtkSmartPointer<vtkPolyData>::New();
            vtkSmartPointer<vtkPoints> left_points = vtkSmartPointer<vtkPoints>::New();
            left_points->DeepCopy(cloud_translated);
            left_source->SetPoints(left_points);          //设置点坐标
            left_source->SetVerts(vertices);

             //重新加入点集
             cloud_index->Reset();
             for (int seg_order = 1;seg_order < SelectPoints.size();seg_order++)   //按顺序加入点簇
             {
                 for (std::vector<reference>::iterator it = SelectPoints.at(seg_order).begin(); it != SelectPoints.at(seg_order).end(); it++)
                 {
                     cloud_index->InsertNextPoint(it->point);      //累计记录所选点簇
                 }
             }
             {


                 PTPlaneICP* icp_finall = new PTPlaneICP();
                 icp_finall->setNormals(stl_target);
                 icp_finall->setEuclideanFitnessEpsilon(0.00001);//误差限，若超过该限制，则视为不收敛
                 icp_finall->setMaximumIterations(50);           //最大迭代次数，默认为100，这里设为300时收敛结果也一样
                 icp_finall->setInputSource(cloud_index);        //source经过变换到target,探针获取的特征点
                 icp_finall->setInputTarget(cloud_model);        //target不变，CT模型
                 icp_finall->align(Finall, OptMatrix);  //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵==(先应用粗配准)

                OptMatrix = icp_finall->getFinalTransformation();
                OptError = icp_finall->getFitnessScore(); //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
                std::cout<<"最终的误差为："<<OptError<<std::endl;    //经过PTP之后有可能误差反而变大
             }
            iterations_cnt = ShowSave(); //更新点云_translated ,并分析取得iterations_cnt

            //构建待绘制点云的PolyData
            vtkSmartPointer<vtkPolyData>  right_source= vtkSmartPointer<vtkPolyData>::New();
            vtkSmartPointer<vtkPoints> right_points = vtkSmartPointer<vtkPoints>::New();
            right_points->DeepCopy(cloud_translated);
            right_source->SetPoints( right_points);          //设置点坐标
            right_source->SetVerts(vertices);

            //mapper
            vtkSmartPointer<vtkPolyDataMapper> mapper_left = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper_left->SetInputData(left_source);

             vtkSmartPointer<vtkPolyDataMapper> mapper_right= vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper_right->SetInputData(right_source);
           //actor
             vtkSmartPointer<vtkActor> actor_left = vtkSmartPointer<vtkActor>::New();
            actor_left->SetMapper(mapper_left);
            actor_left->GetProperty()->SetColor(1.0, 0, 0); //白色  原始target

             vtkSmartPointer<vtkActor> actor_right = vtkSmartPointer<vtkActor>::New();
            actor_right->SetMapper(mapper_right);
            actor_right->GetProperty()->SetColor(1.0, 0, 0);     //绿色   粗配准结果 （1.0，0，0）红色

            //render
            renderer_1->RemoveActor(renderer_1->GetActors()->GetLastActor()); //删除掉tough的点云
            renderer_2->RemoveActor(renderer_2->GetActors()->GetLastActor());

            renderer_1->AddActor(actor_left);
            renderer_2->AddActor(actor_right);


            RW->SetWindowName("Incremental");
            RW->SetWindowName("Incremental");

            RW->Render();
            RWInteractor->Start();
        }
    }
    //关闭当前窗口
    SelectPoints.clear();
    SelectPoints  =  pre_SelectPoints;  //还原选择的点集SelectPoint，便于再次进行实验
    std::cout<<SelectPoints.size()<<std::endl;
    return 1;
}


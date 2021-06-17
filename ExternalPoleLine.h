#ifndef TSPSOLVER_H
#define TSPSOLVER_H

#include"myicp_vtk.h"
#include "SimulatedAnnualingSolver.h"
#include <random>
#include <math.h>

#define R_Range 10.0
#define T_Range 3.0

//终止条件，连续5次外循环没有更新最佳解
class SAICP:public ICPBASE,public SimulatedAnnualingSolver     //继承传统配准方法
{
public:
    explicit SAICP(int rand_seed);
    void setCenterMatrix(const Eigen::Matrix4f& spacetoPlane);
    void setRangeofAngle(float angle);

    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess) override; //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵
    virtual void Analysis()override;
    virtual ~SAICP(){}

    vtkSmartPointer<vtkKdTree>  target_kdtree;      //创建kd_tree对象
protected:
    Eigen::Matrix4f SpacetoPlane;         //通过拟合平面和中心得到的模型旋转中心，该矩阵表示从空间坐标系到自身坐标系的转换矩阵
private:
    float RangeofAngle;            //角度的震荡范围
    Eigen::Matrix4f ToughMatirx;   //给定的粗配准矩阵


protected:
    double take_step(double step_size) override;
    void undo_step() override;
    void save_best() override;
    double energy() override;
    void print() override;
    void SetBest() override;
private:
    double PreGtArray[3];
    double BestGtArray[3];
    double NowGTArray[3];
    double MaxParament[3];     //绕轴旋转的角度最大值、平移的最大值
    double MinParament[3];
    void GetRang();
    void updateRT();
    void Cauchy(double max, double min, double& temp_m);
     std::mt19937 m_randGen;
};

int sgn(double d) { return d<0 ? -1 : d>0; }   //符号函数

SAICP::SAICP(int rand_seed):SimulatedAnnualingSolver(rand_seed)
  ,target_kdtree(vtkSmartPointer<vtkKdTree>::New())
{
    std::random_device r;
    std::seed_seq seed2{ r(), r(), r()};
    m_randGen.seed(seed2);                                      //用种子构建随机生成器
    GetRang();
}
void SAICP::save_best()                                     //最优解获得更新时，保存当前矩阵
{
    for (int i = 0;i < 3;i++)
    {
        BestGtArray[i] = NowGTArray[i];
    }
}
void SAICP::SetBest()
{
    for (int i = 0;i < 3;i++)
    {
        NowGTArray[i] = BestGtArray[i];
    }
    updateRT();
}

void SAICP::print()
{

}
//@step_index: 记录当前步骤时内循环的第几次
double SAICP::take_step(double step_index)                       //更新矩阵，将更新前的矩阵保存在PreGtArray
{
    //还原ICP前的矩阵的旋转
//     Eigen::Matrix4f Reduction = SpacetoPlane*(finall_transform*ToughMatirx.inverse())*HomoInverse(SpacetoPlane);
//     Eigen::Quaternionf temp_Q = RotationMatrix2Quaternion(Reduction.block(0,0,3,3));
//     float z_angle,y_angle,x_angle;
//     toEulerAngle(temp_Q,x_angle,y_angle,z_angle);

    double ratio = (double)((m_iters_fixed_T-step_index)/m_iters_fixed_T);
    double max_parament, min_parament;
    for (int i = 0;i < 3;i++)
    {
        max_parament = ratio*MaxParament[i];
        min_parament = ratio*MinParament[i];
        PreGtArray[i] = NowGTArray[i];
        Cauchy(max_parament, min_parament, NowGTArray[i]);
         //std::cout<<NowGTArray[i]<<std::endl;
    }
    //将最终的扰动矩阵施加在当前的最优矩阵中
    updateRT();    //将NowGTArray应用在ToughtMatrix上
    double result = energy();   //应用当前矩阵，再进行ICP
    return result;
}

void SAICP::undo_step()                                       //回退到上一个状态，即读取前驱矩阵
{
    NowGTArray[0] = PreGtArray[0];
    NowGTArray[1] = PreGtArray[1];
    NowGTArray[2] = PreGtArray[2];
//    NowGTArray[3] = PreGtArray[3];
//    NowGTArray[4] = PreGtArray[4];
//    NowGTArray[5] = PreGtArray[5];
}

double SAICP::energy()
{
    float sum_error=0.0;
    //1、直接将当前的矩阵作为最终矩阵进行误差计算

//      //若查找树没有构建，就构建一个树
//      if(target_kdtree->GetNumberOfRegions()==0)
//      {
//          target_kdtree->BuildLocatorFromPoints(target_);
//      }
//      //将当前的最优矩阵应用在Cloud中
//      vtkSmartPointer<vtkPoints> input_transformed = vtkSmartPointer<vtkPoints>::New();
//       vtk::transformation<float>(input_, input_transformed, finall_transform);

//      double *testPoint;    //待查找点
//      vtkSmartPointer<vtkIdList> index = vtkSmartPointer<vtkIdList>::New();
//      int number =input_transformed->GetNumberOfPoints();
//      for (int i = 0;i < number;i++)
//      {
//          testPoint = input_transformed->GetPoint(i);
//          target_kdtree->FindClosestNPoints(1, testPoint, index);

//          vtkIdType point_ind = index->GetId(0);
//          double *goal = target_->GetPoint(point_ind);

//          float temp_distance = std::sqrt(std::pow((testPoint[0]-goal[0]),2)+std::pow((testPoint[1]-goal[1]),2)+std::pow((testPoint[2]-goal[2]),2));
//          sum_error+=temp_distance;
//      }

   MyICP  icp_1 ;
   icp_1.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
   icp_1.setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
   icp_1.setInputSource(input_);        //source经过变换到target,探针获取的特征点
   icp_1.setInputTarget(target_);        //target不变，CT模型
   vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
   icp_1.align(nonsense, finall_transform);
   sum_error = icp_1.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了

    return sum_error;
}
//也是生成随机数，再按照设定的函数得出震荡后的值，因此也有可能重复
void SAICP::Cauchy(double max, double min, double& temp_m)
{
    double result_m = 0;
    while(1)
    {
        std::uniform_real_distribution<> dis(0, 1);
        double u = dis(m_randGen);
        double yi = T*sgn(u - 0.5)*(pow(1 + 1 / T, abs(2 * u - 1)) - 1);   //温度越低，搜索范围越小------所以其实温度也起到了逐步减小震荡角度的作用，0.5为均值
        result_m = temp_m + (max - min)*yi;
        if (result_m > min&&result_m < max)                  //要保证新生成的参数也在取值范围内
        {
            break;
        }
    }
    temp_m = result_m;
}
void SAICP::GetRang()
{
    for (int i = 0;i < 3;i++)
    {
        MaxParament[i] = R_Range;
        MinParament[i] = -R_Range;
        NowGTArray[i] = 0.0;
    }
//    for (int i = 3;i < 6;i++)
//    {
//        MaxParament[i] = T_Range;
//        MinParament[i] = -T_Range;
//        NowGTArray[i] = 0.0;
//    }
}
void SAICP::updateRT()
{
    float x_angle = NowGTArray[0]*M_PI/180.0;
    float y_angle = NowGTArray[1]*M_PI/180.0;
    float z_angle = NowGTArray[2]*M_PI/180.0;

    finall_transform = RotateZ(NowGTArray[2])*RotateY(NowGTArray[1])*RotateX(NowGTArray[0]);
    finall_transform = HomoInverse(SpacetoPlane)*finall_transform*SpacetoPlane;     //将扰动施加在以自己为旋转中心的坐标系中

    Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
   // tran.block(0,3,3,1)<<NowGTArray[3],NowGTArray[4],NowGTArray[5];
    //std::cout<<tran<<std::endl;

    finall_transform = tran*finall_transform;

//    Eigen::Matrix4f Reduction = SpacetoPlane*finall_transform*HomoInverse(SpacetoPlane);
//    Eigen::Quaternionf temp_Q = RotationMatrix2Quaternion(Reduction.block(0,0,3,3));
//    float z_angle,y_angle,x_angle;
//    toEulerAngle(temp_Q,x_angle,y_angle,z_angle);


    finall_transform = finall_transform*ToughMatirx;
}

void SAICP::align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess)
{
    if (isinit == false)      //初始化步骤，这里是初始化input_transformed变量
    {
        target_kdtree->BuildLocatorFromPoints(target_);
        isinit = true;

    }
    ToughMatirx = guess;
    finall_transform = ToughMatirx;

    solve();    //得到最优的array，存储在bestAarry中,并更新final_transform

    MyICP  icp_1 ;
    icp_1.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
    icp_1.setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
    icp_1.setInputSource(input_);        //source经过变换到target,探针获取的特征点
    icp_1.setInputTarget(target_);        //target不变，CT模型
    vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
    icp_1.align(nonsense, finall_transform);
    EMS = icp_1.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
    finall_transform = icp_1.getFinalTransformation();
}
void SAICP::setCenterMatrix(const Eigen::Matrix4f& spacetoPlane)
{
     SpacetoPlane = spacetoPlane;
}
void SAICP::setRangeofAngle(float angle)
{
    RangeofAngle = angle;
}


void SAICP::Analysis()
{

    float guess_error = 10000.0;
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
#endif // TSPSOLVER_H
#pragma once

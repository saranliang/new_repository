#include"icp_base.h"
//以四个点为单元进行配准，这里的方法是使用基于方向一致性约束
void ICPBASE::setMaximumIterations(int number)
{
    MaximumIteration_ = number;
}
void ICPBASE::setTransformationEpsilon(double error)
{
    transformation_epsilon_ = error;
}
void ICPBASE::setEuclideanFitnessEpsilon(float error)
{
    FitnessEpsilon_ = error;
}

void ICPBASE::setInputSource(const vtkSmartPointer<vtkPoints> &cloud)
{
    input_ = cloud;
    M = static_cast<uint32_t>(input_->GetNumberOfPoints());
}
void ICPBASE::setInputTarget(const vtkSmartPointer<vtkPoints> &cloud)
{
    target_ = cloud;
}
 void ICPBASE::align(vtkSmartPointer<vtkPoints> cloud)
{
    align(cloud, Eigen::Matrix4f::Identity());
}

bool ICPBASE::hasConverged()
{
    return isConverged;
}
//返回的是均方根误差
float ICPBASE::getFitnessScore()
{
    return EMS;
}

Eigen::Matrix4f ICPBASE::getFinalTransformation()
{
    return finall_transform;
}

void ICPBASE::Init()
{
    input_transformed = vtkSmartPointer<vtkPoints>::New();    //为input_transformed初始化，分配内存
}
ICPBASE:: ~ICPBASE(){};   //纯虚函数也要为其给出实现

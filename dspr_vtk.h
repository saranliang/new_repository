#ifndef DSPR_VTK_H
#define DSPR_VTK_H

#include"icp_base.h"
#include"myicp_vtk.h"
#include"MathTool.h"
class DSPR:public ICPBASE     //继承传统配准方法   --子类必须包含父类的头文件
{
    public:
    DSPR():target_kdtree(vtkSmartPointer<vtkKdTree>::New())
    {	}
    void setCenterMatrix(const Eigen::Matrix4f& spacetoPlane);
    void setRangeofAngle(float angle);
    void setPara(std::string RobostName,float p);   //设置矩阵估计时的鲁棒函数

    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess); //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵
    virtual void Analysis();
    virtual ~DSPR(){}
protected:  
    virtual void Perturb( Eigen::MatrixXf& perturbTrans,int numPertub,float Ratio);                 //震荡模型
    virtual void DeterminePerturb( Eigen::MatrixXf& perturbTrans,int numPerturb,float Angle); //均匀分布的震荡模型
    virtual bool FindOptPerturb(const Eigen::MatrixXf&  perturbTran,Eigen::Matrix4f& optPerturn,Eigen::Matrix4f souece, const Eigen::Matrix4f& guess,float& finallEMS);    //找到最优震荡结果

    Eigen::Matrix4f SpacetoPlane;         //通过拟合平面和中心得到的模型旋转中心，该矩阵表示从空间坐标系到自身坐标系的转换矩阵
    float RangeofAngle;            //角度的震荡范围
    vtkSmartPointer<vtkKdTree>  target_kdtree;      //创建kd_tree对象
    Parameters Para;          //鲁棒函数
};

#endif // DSPR_VTK_H

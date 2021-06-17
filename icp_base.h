#ifndef ICP_BASE_H
#define ICP_BASE_H

#include<string>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkNew.h>

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
//抽象类
class ICPBASE
{
public:
    ICPBASE():             //抽象类也可以有构造函数
        isinit(false)
        ,iteration_k(0)
        ,isConverged(false)
        , MaximumIteration_(300)
        , transformation_epsilon_(1e-4)
        , FitnessEpsilon_(0.001)
        , EMS(10000.0) //使用默认构造函数，为无权值的，即均匀分布
    {	}
    virtual void setMaximumIterations(int number);          //最大迭代次数，默认为10，这里设为300时收敛结果也一样
    virtual void setTransformationEpsilon(double error);    //前后两次变换矩阵的的最大允许差异，不满足时收敛终止
    virtual void setEuclideanFitnessEpsilon(float error);

    virtual void setInputSource(const vtkSmartPointer<vtkPoints> &cloud);         //source经过变换到target
    virtual void setInputTarget(const vtkSmartPointer<vtkPoints> &cloud);            //target不变
    virtual void align(vtkSmartPointer<vtkPoints> cloud);
    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess)=0; //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵
    virtual bool hasConverged();//匹配是否正确
    virtual float getFitnessScore();                              //匹配误差多少
    virtual Eigen::Matrix4f getFinalTransformation(); //输出旋转匹配矩阵

    virtual void Analysis()=0;   //纯虚函数，需要被重写
    virtual ~ICPBASE()=0;

protected:
    virtual void Init();

    vtkSmartPointer<vtkPoints> input_;
    vtkSmartPointer<vtkPoints> target_;
    vtkSmartPointer<vtkPoints> input_transformed;     //转换后的模型点云
    Eigen::Matrix4f finall_transform;        //最终变换

    int MaximumIteration_;                  //最大迭代次数
    double transformation_epsilon_;         //矩阵变换差
    float FitnessEpsilon_;                  //前后误差的阈值
    float EMS;                              //最终的误差

    bool isinit;
    int M;                   //input_点的个数
    unsigned int iteration_k;          //记录迭代次数
    bool isConverged;         //记录是否收敛

};

#endif // MYICP_VTK_H


#ifndef MYICP_VTK_H
#define MYICP_VTK_H
// PCL includes
#include<string>
#include"Robost.h"
#include <vtkKdTree.h>
#include <vtkPoints.h>
#include <boost/shared_ptr.hpp>
#include"icp_base.h"

struct Correspondence
{
    /** \brief Index of the query (source) point. */
    int index_query;
    /** \brief Index of the matching (target) point. Set to -1 if no correspondence found. */
    int index_match;

    double match_point[3];
    /** \brief Distance between the corresponding points, or the weight denoting the confidence in correspondence estimation */
    union
    {
      float distance;
      float weight;
    };
};
//vector定义的标准方法：为了提高运算速度，对于SSE或者AltiVec指令集，向量化必须要求向量是以16字节即128bit对齐的方式分配内存空间，因此需要使用Eigen提供的内存分配方法
typedef std::vector<Correspondence, Eigen::aligned_allocator<Correspondence> > Correspondences;
//重写PCL中的对应点查找模块
class CorrespondenceEstimationMY
{
public:
    virtual void setInputTarget(vtkSmartPointer<vtkPoints>);
    virtual void setInputSource(vtkSmartPointer<vtkPoints>);
    virtual void determineCorrespondences(Correspondences &correspondences, float& Error);
    virtual  ~CorrespondenceEstimationMY(){}
protected:
    vtkSmartPointer<vtkPoints> input_;
    vtkSmartPointer<vtkPoints> target_;
    vtkSmartPointer<vtkKdTree> tree_;
};

//模拟PCL接口重写的ICP方法
class MyICP : public ICPBASE
{
public:
    MyICP():
        correspondence_(new Correspondences)
       // ,correspondence_estimation(new CorrespondenceEstimationMY)  //对应点估计法已经被初始化了
        ,correspondence_estimation(new CorrespondenceEstimationMY)  //对应点估计法已经被初始化了
        ,Para(Parameters())     //使用默认构造函数，为无权值的，即均匀分布
    {	}
    void setPara(std::string RobostName,float p);   //设置矩阵估计时的鲁棒函数
    void setCorrespondence_estimation(std::string Correspondence_estimationName);           //设置对应估计方法
    void setSetSize(const std::vector<int>& setSize);
    Parameters Para;          //鲁棒函数

    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess); //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵
    virtual void transform_es_svd_weight();
    virtual void Analysis();
    virtual ~MyICP(){}  //作为子类需要实现父类的纯虚函数
protected:
    Eigen::Matrix4f try_transform;           //单次对应点的计算
    Eigen::Matrix4f pre_transform;           //上次的最终变化   
    CorrespondenceEstimationMY* correspondence_estimation;
    boost::shared_ptr<Correspondences> correspondence_;  
    Eigen::VectorXf Weight;    //点对权值  

    std::vector<int> SetSize;
    float CountError();        //计算误差

};

#endif // MYICP_VTK_H

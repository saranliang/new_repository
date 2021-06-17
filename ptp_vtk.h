#ifndef PTP_VTK_H
#define PTP_VTK_H

#include"myicp_vtk.h"
//类的默认继承方式是保护继承:基类的公有和保护继承为保护，只能由派生类和友元函数访问；私有类继承为私有，不可见

class PTPlaneICP :public  MyICP     //继承传统配准方法
{
public:
    PTPlaneICP()
    {
        //1、多态：将父类指针指向子类的对象，子类可以根据自己的情况重写父类的virtual函数，但是该指针在外部不能调用子类的中独有的变量
        //好处：可以使用统一的编程方法和接口来编程
        //也可以实现----强制类型转换：(pcl::registration::CorrespondenceEstimationMy<pcl::PointXYZ, pcl::PointXYZ>::Ptr)	correspondence_estimation
        //2、使用同名子类变量来覆盖父类变量，好处：可以添加在子类中添加新的成员变量
        //默认使用传统对应点查找作为配对方法
        //correspondence_estimation = pcl::registration::CorrespondenceEstimationDC<pcl::PointXYZ, pcl::PointXYZ>::Ptr(new pcl::registration::CorrespondenceEstimationDC<pcl::PointXYZ, pcl::PointXYZ>);
       //if(CorrMode =="DC")  correspondence_estimation = pcl::registration::CorrespondenceEstimationDC<pcl::PointXYZ, pcl::PointXYZ>::Ptr(new pcl::registration::CorrespondenceEstimationDC<pcl::PointXYZ, pcl::PointXYZ>);
       //if(CorrMode =="FP ") correspondence_estimation = pcl::registration:: CorrespondenceEstimationFP <pcl::PointXYZ, pcl::PointXYZ>::Ptr(new pcl::registration:: CorrespondenceEstimationFP<pcl::PointXYZ, pcl::PointXYZ>);
    }
    void transform_es_svd_weight();                                     //重写矩阵估计函数
    void setNormals(vtkSmartPointer<vtkPolyData> STLCloud); //设置法向量
    vtkSmartPointer<vtkPolyData> target_normals;           //同时包含数据点和对应的法向量
protected://
    //设置同名配准方法，不使用多态，从而可以在自定义的对应点方法内添加新的成员变量---错误
    //由于该变量在子类中定义，初始化时也是子类初始化，因此父类的指针中访问不到该标量，
    //pcl::registration::CorrespondenceEstimationDC<pcl::PointXYZ, pcl::PointXYZ>::Ptr correspondence_estimation;

};
void estimateRigidTransformation_ptp(const vtkSmartPointer<vtkPoints> source,const vtkSmartPointer<vtkPolyData> target,std::vector<float>::iterator weights_it,Eigen::Matrix4f& transformation_matrix);


#endif // PTP_VTK_H

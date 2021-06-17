#ifndef START_H
#define START_H
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <map>
#include<vtkPoints.h>
#include<vtkPolyData.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include<vtkKdTree.h>

//在一个头文件中写extern，声明该变量
//1.h 写extern  1.cpp定义     2.cpp想使用时包含1.h即可以（     通过在1.h只声明不定义后，避免多次包含1.h后出现重复包含的情况）

//存储 reference 点
struct reference
{
    reference()
    {

    }
    float point[3];
    int index;                 //指示在完整target_cloud中的序号
};


extern int ModePointSelect;     //选点事件对应的操作函数：1、新增点 2、修改时选中点 3、移动修改点 4、增加选中的点
extern std::string ModeDeviation;   //设置偏移点的方法
extern int Tough;               //自定义偏移时的粗配准参考点随机数
extern int Fine;                //自定义偏移时的精配准参考点随机数
extern std::string RegistrationOrder_1;  //配准指令左
extern std::string RegistrationOrder_2;  //配准指令右
extern std::string FILENAME;    //保存旋转后的source、refrence、配准结果等文件的所在的文件路径前缀
extern std::string OPENROUTE;    //打开target、摄像参数、中心点拟合等文件的路径

extern vtkSmartPointer<vtkPoints> cloud_choosed;        //鼠标选中的簇，主要用来显示
extern vtkSmartPointer<vtkPoints> new_cloud_choosed;    //选中点的cloud备份
extern vtkSmartPointer<vtkPoints> cloud_target;         //读取pcd的模拟旋转后的点云
extern vtkSmartPointer<vtkPolyData> stl_target;         //存储stl格式的target文件

extern std::vector<std::vector<reference>> SelectPoints;    //保留簇关系的容器，用于后序计算，通过下标和对应的三维点坐标记录
extern std::vector<reference> OneArea;                      //set内部只保留不重复的元素
//extern std::vector<std::map<int,pcl::PointXYZ>> NewPoints;       //选择点的vector备份

extern int IsNext;   //表示是否接着进行精配准0：初始，1：精配准，2：复合配准
extern bool IsClose;  //标识当前次测试是否结束


extern float XCAngle;
extern float YCAngle;
extern float ZCAngle;

extern float XRAngle;
extern float YRAngle;
extern float ZRAngle;

extern float XRDistance;
extern float YRDistance;
extern float ZRDistance;
extern bool IsCenterRotation;

extern double SampleRate;

class KeyBoardInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static KeyBoardInteractorStyle* New();  //对于继承自vtkObjectBase的类，必须覆盖New()来生成一个实际类的对象，而不是得到一个vtkObjectBase对象
  vtkTypeMacro(KeyBoardInteractorStyle, vtkInteractorStyleTrackballCamera);  //通过该宏，覆盖vtk的基本函数
  virtual void OnKeyPress() override;
private:
};

class RegistrationSatrt
{
public:
     RegistrationSatrt(vtkSmartPointer<vtkPoints> target);
     int start();
private:
     vtkSmartPointer<vtkPoints> cloud_model;     //模型点云
     vtkSmartPointer<vtkPoints> cloud_scan;      //模拟旋转后的点云
     vtkSmartPointer<vtkPoints> cloud_index;     //探针选取的点（带噪声）
     vtkSmartPointer<vtkPoints> cloud_index_tough;//选取的点执行粗变换
     vtkSmartPointer<vtkPoints> cloud_index_translated; //选取的点执行精配准变换
     vtkSmartPointer<vtkPoints> cloud_tough;//旋转后的点云执行粗+精变换
     vtkSmartPointer<vtkPoints> cloud_translated;//模拟旋转后的点云执行精配准变换

     float OptError;
     Eigen::Matrix4f OptMatrix;
     Eigen::Matrix4f ToughMatrix;

     Eigen::Matrix3f test_m;
     Eigen::Vector3f sta_euler ;          //表示旋转的顺序是z、x、y,是基于右手准则的右乘法,旋转四元数。
     Eigen::Vector3f exp_euler ;
     Eigen::MatrixXf std_error ;
     Eigen::MatrixXf expr_error;

     Eigen::Matrix4f FossaCenter;       //通过拟合平面和中心得到的模型旋转中心，该矩阵表示从空间坐标系到自身坐标系的转换矩阵

     vtkSmartPointer<vtkKdTree>  target_kdtree;      //创建kd_tree对象

     std::string ModeTrans ;       //矩阵估计的方法
     std::string ModePara ;        //权值函数
     float ParameterValue ;     //权值函数对应的参数
     std::string ModeCorr ;        //匹配点对的方法


    //函数只声明，不定义：因为函数的声明可以被多次包含，而定义被多次包含时就会出错
    //计算两点云的整体点云误差
    void CloudError(vtkSmartPointer<vtkPoints> Target, vtkSmartPointer<vtkPoints> TraSource,float& Sum);

    //分析验证点云的误差情况
    float AnalysisError();

    //分析给定点集在当前最优矩阵中的误差
    float CurrentError(vtkSmartPointer<vtkPoints> Cloud);

    //map中的比较函数，需要设置为全局函数或者类内静态函数，这和回调函数相同
    static bool cmp(int x, int y);

    //绕髋臼窝平面的中心法线旋转N度
    Eigen::Matrix4f FossaTransformation(float Angle);
    //主程序开始

    //配准指令执行
    void  RegistrationImplement(std::string RegistrationOrder);
    std::string  ShowSave();
    std::string DecompositionError(const Eigen::Matrix4f& GoalMatrix);
};



#endif // START_H

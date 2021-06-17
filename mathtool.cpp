#include"MathTool.h"
//函数定义一定要在cpp文件中，不然重复包含.h文件时会出错
//求由旋转平移的构建的齐次矩阵的逆矩阵
Eigen::Matrix4f HomoInverse(const Eigen::Matrix4f& Source)
{
     Eigen::Matrix4f real_result = Eigen::Matrix4f::Identity();
     real_result.block(0,0,3,3) =  Source.block(0,0,3,3).transpose();
     real_result.block(0,3,3,1) = -1.0*(Source.block(0,0,3,3).transpose()*Source.block(0,3,3,1));
     return real_result;
}

//拟合空间平面
//https://blog.csdn.net/konglingshneg/article/details/82585868
//克拉默法则行列式求解 ：https://my.oschina.net/u/4266515/blog/3329781
//最小二乘解：https://blog.csdn.net/qq_40335930/article/details/100546496?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~aggregatepage~first_rank_v2~rank_aggregation-7-100546496.pc_agg_rank_aggregation&utm_term=c%2B%2B+%E6%8B%9F%E5%90%88%E5%B9%B3%E9%9D%A2&spm=1000.2123.3001.4430
//SVD分解：    https://blog.csdn.net/qq_37569355/article/details/112620901?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~aggregatepage~first_rank_v2~rank_aggregation-2-112620901.pc_agg_rank_aggregation&utm_term=c%2B%2B+%E6%8B%9F%E5%90%88%E5%B9%B3%E9%9D%A2&spm=1000.2123.3001.4430
// Para:A,B,C,D       Ax+By+Cz+1=0
Eigen::MatrixXf FitPlane(vtkSmartPointer<vtkPoints> fit_cloud)
{
       //直接调用最小二乘公式
       int number = fit_cloud->GetNumberOfPoints();
        Eigen::MatrixXf M(number,3);
        Eigen::MatrixXf X(3,1);
        Eigen::MatrixXf N(number,1);
        double fit_point[3];
        for(int i=0;i<number;i++)
        {
            fit_cloud->GetPoint(i,fit_point);

            M(i,0) = static_cast<float>(fit_point[0]);
            M(i,1) = static_cast<float>(fit_point[1]);
            M(i,2) = static_cast<float>(fit_point[2]);
            N(i,0) = -1.0;
        }
        X =(M.transpose()*M).inverse()*M.transpose()*N;
        return X;
}
//点到平面的投影 target = source-k*normal
//@Para :平面法向量参数
//@source:待投影空间点
//@测试投影后的点是否真的在平面上：OK
Eigen::Vector3f ProjectiToPlane(const Eigen::Vector3f& Para,const Eigen::Vector3f& source)
{

    //这里是计算投影点，因此不能单独将法向量归一化，而是要将其与D一起作为平面方程的整体
    float A =Para(0);
    float B =Para(1);
    float C = Para(2);
    //float distance  = (A*source(0)+B*source(1)+C*source(2)+1.0)/sqrt(A*A+B*B+C*C);
     float distance  = (A*source(0)+B*source(1)+C*source(2)+1.0)/sqrt(A*A+B*B+C*C);
    Eigen::Vector3f norm_para = Para.normalized();         //normalize()是将调用这单位化，无返回
                                                                                                               //normalized()是返回调用者的单位化，调用者不改变
   Eigen:: Vector3f result;
   result =  source-distance*norm_para;                               //投影点= 原点-单位向量*点到平面的距离
    // Eigen:: Vector3f test(1.0,1.0,(-1.0-A-B)/C);
    // std::cout<<(result-test).dot(Para)<<std::endl;
   return result;
}
//从空间坐标转换到平面坐标
//@PlanetoSpace： 空间坐标 = PlanetoSpace*平面坐标
//@origin:人为设定的转换后的坐标原点,默认为原来的原点投影点    (默认参数只能在声明或定义其中之一指明，都写就出错)
void  PlaneCoordinate(const Eigen::Vector3f& Para,Eigen::Matrix4f& PlanetoSpace, Eigen::Vector3f Origin = Eigen::Vector3f(0.0,0.0,0.0) )
{
    Eigen::Vector3f origin = Origin;
     Eigen::Vector3f random_p(-97.827, 175.288, 953.775); //人为指定x轴的方向
    origin = ProjectiToPlane(Para,origin);                     //413,111,324
    random_p = ProjectiToPlane(Para,random_p);  //319,100,448

    Eigen::Vector3f  x_axis = random_p-origin;
    Eigen::Vector3f   y_axis = Para.cross(x_axis);               //Eigen中默认是左手坐标系，但是cross后方向的确定使用的是右手坐标系
    //Eigen::Vector3f   y_axis = x_axis.cross(Para);            //两种叉乘计算的结果是一样的
    x_axis = x_axis.normalized();
    y_axis = y_axis.normalized();
     Eigen::Vector3f  z_axis = Para.normalized();

     Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

     result.block(0,0,3,1) = x_axis;
     result.block(0,1,3,1) = y_axis;
     result.block(0,2,3,1) = z_axis;
     result.block(0,3,3,1)=origin;
     //该矩阵是从平面坐标到空间坐标的齐次变换矩阵，即 空间坐标 = result*平面坐标
     //因此需要该齐次变换矩阵的逆矩阵
    //  Eigen::Matrix4f real_result = Eigen::Matrix4f::Identity();
    //  real_result.block(0,0,3,3) =   result.block(0,0,3,3).transpose();
    //  real_result.block(0,3,3,1) = -1.0*(result.block(0,0,3,3).transpose()*result.block(0,3,3,1));

    PlanetoSpace = result;
    //SpacetoPlane = real_result;

}
//平面上拟合圆
//@fit_points:待拟合点集（点维数*点个数）    =向量都是作为一列来存储的
//@Para:(圆心x坐标，圆心y坐标，半径)
void FitCircle(const Eigen::MatrixXf& fit_points,  Eigen::Vector3f& Para)
{
    int number = fit_points.cols();
    Eigen::MatrixXf A(number,3);
    Eigen::MatrixXf X(3,1);
    Eigen::MatrixXf B(number,1);
    A.block(0,0,number,2) = fit_points.transpose();
    A.block(0,2,number,1)  = -1.0*Eigen::MatrixXf::Ones(number,1);
    for(int i=0;i<number;i++)
    {
           B(i,0) = fit_points(0,i)*fit_points(0,i)+fit_points(1,i)*fit_points(1,i);
    }
    X = (A.transpose()*A).inverse()*A.transpose()*B;
    Para(0) = X(0,0)/2.0; //横坐标
    Para(1) = X(1,0)/2.0; //纵坐标
    Para(2) = std::sqrt(Para(0)*Para(0)+Para(1)*Para(1)-X(2,0)); //半径
}

//根据空间点拟合对应圆的圆心及对应的位姿变换矩阵
Eigen::MatrixXf FitCircle3D(const vtkSmartPointer<vtkPoints> fit_cloud,  Eigen::Vector3f& Origin)
{
     Eigen::MatrixXf plane_para = FitPlane(fit_cloud);  //拟合后的平面参数
     Eigen::Matrix4f PlanetoSpace;
     Eigen::Matrix4f SpacetoPlane ;
    PlaneCoordinate(plane_para,PlanetoSpace);   //该平面所在的坐标系姿态矩阵
    SpacetoPlane = HomoInverse(PlanetoSpace); //求该姿态的反矩阵
    int point_number = fit_cloud->GetNumberOfPoints();
     Eigen::MatrixXf fit_points(2,point_number); //转为2维平面坐标的点集
     Eigen::MatrixXf plane_point(4,1);
    Eigen::MatrixXf space_point(4,1);
    double fit_point[3];
    for(int i=0;i<point_number;i++)
    {
        fit_cloud->GetPoint(i,fit_point);

        space_point(0,0) = static_cast<float>(fit_point[0]);
        space_point(1,0) = static_cast<float>(fit_point[1]);
        space_point(2,0) = static_cast<float>(fit_point[2]);
        space_point(3,0) = 1.0;
        plane_point = SpacetoPlane* space_point;
        fit_points.block(0,i,2,1) = plane_point.block(0,0,2,1);         //只取转换后的平面坐标系中坐标的x值和y值
        //std::cout<< plane_point.block(0,0,3,1).transpose()<<std::endl;
     }
     Eigen::Vector3f circle_para;
     FitCircle( fit_points,circle_para);    //拟合平面中的圆心与半径
     //std::cout<<"圆心："<<circle_para<<std::endl;

     plane_point(0,0) = circle_para(0);
     plane_point(1,0) = circle_para(1);
    plane_point(2,0) = 0.0;
    plane_point(3,0) = 1.0;
    space_point = PlanetoSpace*plane_point; //圆心在三维空间中的坐标
    Origin = space_point.block(0,0,3,1);

   //利用圆心将点云所在坐标系转换到以圆心为原点的坐标系
   PlaneCoordinate(plane_para,PlanetoSpace,Origin);   //该平面所在的坐标系姿态矩阵
   SpacetoPlane = HomoInverse(PlanetoSpace); //求该姿态的反矩阵
    return SpacetoPlane;
}

//旋转平移矩阵定义------------------------------------------------------------------------------------------------
//运算过程都是齐次坐标
//连乘矩阵可以任意加括号
//绕X轴正向旋转Angle角度的旋转矩阵
Eigen::Matrix4f RotateX(float Angle)
{
    Eigen::MatrixXf m(4, 4);
    m << 1, 0, 0, 0,
        0, cos(Angle), -sin(Angle), 0,
        0, sin(Angle), cos(Angle), 0,
        0, 0, 0, 1;
    return m;
}
//绕Y轴正向旋转
Eigen::Matrix4f RotateY(float Angle)
{
    Eigen::Matrix4f m(4, 4);
    m << cos(Angle), 0, sin(Angle), 0,
        0, 1, 0, 0,
        -sin(Angle), 0, cos(Angle), 0,
        0, 0, 0, 1;

    return m;
}
//绕Z轴正向旋转
Eigen::Matrix4f RotateZ(float Angle)
{
    Eigen::MatrixXf m(4, 4);
    m << cos(Angle), -sin(Angle), 0, 0,
        sin(Angle), cos(Angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return m;
}

//平移矩阵
Eigen::Matrix4f Translation(float X, float Y, float Z)
{
    Eigen::Matrix4f m(4, 4);
    m << 1, 0, 0, X,
        0, 1, 0, Y,
        0, 0, 1, Z,
        0, 0, 0, 1;
    return m;
}

//从四元数到旋转矩阵
Eigen::Matrix4f Quaternion2RotationMatrix(const float x, const float y, const float z, const float w)
{
    Eigen::Quaternionf q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Matrix3f R = q.normalized().toRotationMatrix();  //结果是非齐次坐标
                                                            //转换为齐次坐标
    Eigen::Matrix4f RR;
    RR.setZero();        //初始化为0矩阵
    RR.block(0, 0, 3, 3) = R; //左上角3*3为非齐次坐标
    RR(3, 3) = 1.0;
    return RR;
}
//从旋转矩阵到四元数
Eigen::Quaternionf RotationMatrix2Quaternion(Eigen::Matrix3f R)
{
    Eigen::Quaternionf q = Eigen::Quaternionf(R);
    q.normalize();
    return q;
}
void toEulerAngle(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw)
{
// roll (x-axis rotation)
float sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
float cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
float sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
if (fabs(sinp) >= 1)
pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
else
pitch = asin(sinp);

// yaw (z-axis rotation)
float siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
float cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
yaw = atan2(siny_cosp, cosy_cosp);
}

//判断是否是同一个点
bool isSamePoint(float a[3],float b[3])
{
    for(int i=0;i<3;i++)
    {
        if(abs(a[i]-b[i])>0.0001)
        {
            return false;
        }
    }
    return true;
}
//将double数组转换为float数组
void doubletofloat(double a[3],float b[3])
{
    for(int i=0;i<3;i++)
    {
        b[i] = (float)a[i];
    }
}
void savaPointsVTK(vtkSmartPointer<vtkPoints> cloud,const char* name)
{
    vtkSmartPointer<vtkPolyDataWriter> vtkWriter = vtkSmartPointer<vtkPolyDataWriter>::New();
    vtkSmartPointer<vtkPolyData> savaPoly = vtkSmartPointer<vtkPolyData>::New();
    savaPoly->SetPoints( cloud);
    vtkWriter->SetInputData(savaPoly);
    vtkWriter->SetFileName(name);
    vtkWriter->Write();
}
void loadPointsVTK(vtkSmartPointer<vtkPoints>& cloud,const char* name)
{
    cloud->Reset();
    vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
    reader->SetFileName(name);
    reader->Update();

     vtkSmartPointer<vtkPolyData> PD = vtkSmartPointer<vtkPolyData>::New();
     PD = reader->GetOutput();
     cloud = PD->GetPoints();
}

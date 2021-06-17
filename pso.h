#ifndef PSO_H
#define PSO_H
/************************************************************************/
/* Author:      yuzelin       -  https://github.com/AlieYu               */
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include"myicp_vtk.h"
//微粒类                                            --一个微粒 = 一轮
class PARTICLE
{
public:
    double *X;			//微粒的坐标数组             --微粒的坐标即该轮对应的解 = 矩阵参数
    double *V;			//微粒的速度数组            --每一维矩阵参数的变化速度
    double *XBest;		//微粒的最好位置数组         --最优矩阵解(只是针对该轮的计算)
    int    Dim;			//微粒的维数
    double Fit;			//微粒的适合度
    double FitBest;		//微粒最好位置的适合度       --最优矩阵解对应的误差

    //vtkSmartPointer<vtkPoints> Data;              //--每个粒子中的一轮的选点集

    PARTICLE();		    //空构造函数
    PARTICLE(const PARTICLE& CP)   //--自定义深拷贝的构造函数
    {
        Dim = CP.Dim;
        Fit = CP.Fit;
        FitBest = CP.FitBest;
        X=new double[Dim];
        V=new double[Dim];
        XBest=new double[Dim];
        for(int i=0;i<Dim;i++)
        {
            X[i] = CP.X[i];
            V[i] = CP.V[i];
            XBest[i] = CP.XBest[i];
        }

    }
    PARTICLE(int n);    //维数为参数的构造函数

    ~PARTICLE();	    //析构函数

    void SetDim(int d); //设置微粒的维数

};

//群粒子类
//惯性速度是决定收敛速度（过小就慢，过大就无法收敛到极小值，而是在周围游荡）
class PSO
{
protected:
    PARTICLE *Particle; //微粒群数组                    --points
    int    PNum;	    //微粒个数
    int    GBestIndex;	//最好微粒索引                  --该轮对应的矩阵应用到所有轮时误差最小
    double W_max;		//惯性权重的最大值               --初始化 1
    double W_min;		//惯性权重的最小值               --初始化 0.6
    int    IteorMax;	//最大迭代次数

    double C1;			//加速度系数1
    double C2;			//加速度系数2
    double *Xup;		//微粒坐标上界数组              --矩阵每个参数的震荡范围
    double *Xdown;		//微粒坐标下界数组
    double *Vmax;		//微粒最大速度数组              --矩阵每个参数的变化速率最大值  值为50%的震荡范围

    void Initialize();			//初始化群体
    void CalFit();				//计算全体适合度
    virtual void ParticleFly();	//微粒飞翔，产生新一代微粒


    //通讯函数，返回值为false时，系统停止优化

    bool (*Com)(double, /*最优微粒适合度*/ double*, /*最优微粒坐标数组*/
                double**, /*所有微粒坐标指针数组*/ int /*当前最优微粒索引*/ );

public:
    PSO();						//空构造函数
    PSO(int dim,int n);			//dim为微粒维数，n为微粒个数

    ~PSO();						//析构函数

    void SetXup(double*);		//设置微粒坐标上界
    void SetXdown(double*);		//设置微粒坐标下界
    void SetVmax(double*);		//设置微粒最大速度,以数组为参数
    void SetVmax(double);		//设置微粒最大速度，以坐标的上下界的百分比为参数
    void SetC1(double c){C1=c;}	//设置C1
    void SetC2(double c){C2=c;}	//设置C2
    void SetCom(void *p)		//设置通讯函数      --通过函数指针赋值
    {
        Com=(bool(*)(double,double*,double**,int))p;
    }

    void SetIteorMax(int iteor)
    {
        IteorMax=iteor;			//设置最大迭代次数
    }
    //计算特定微粒坐标所对应的适合度，必须由派生类的实际PSO类定义,以便计算适合度
    virtual float GetFit(PARTICLE&)=0;

    PARTICLE& Run(int max);		//按最多次数限制运行PSO   ，返回值是自定义类的引用，通过拷贝构造函数传递给接收的变量
    PARTICLE& Run(double fit);	//按最佳适合度目标运行PSO
    double GetBest(double*);	//获得最佳微粒适合度和坐标

};

//多重继承，先构造ICPBASE,在构造PSO
class PSOICP:public ICPBASE,public PSO
{
public:
    PSOICP(int dim,int n);
    void setCenterMatrix(const Eigen::Matrix4f& spacetoPlane);
    void setRangeofAngle(float angle);

    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess) override; //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵
    virtual void Analysis() override;
    virtual ~PSOICP(){};

    vtkSmartPointer<vtkKdTree>  target_kdtree;      //创建kd_tree对象
protected:
    Eigen::Matrix4f SpacetoPlane;         //通过拟合平面和中心得到的模型旋转中心，该矩阵表示从空间坐标系到自身坐标系的转换矩阵
private:
    float RangeofAngle;            //角度的震荡范围
    Eigen::Matrix4f ToughMatirx;   //给定的粗配准矩阵
    float GetFit(PARTICLE&) override;
};

#endif // PSO_H

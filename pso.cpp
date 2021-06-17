#include"pso.h"
#include"MathTool.h"
//深色的是子类新创建的函数、浅色是继承的函数
PSOICP::PSOICP(int dim,int n):PSO(dim,n)            //构造函数时根据给定的优化参数的维数dim，飞行粒子的个数n
  ,target_kdtree(vtkSmartPointer<vtkKdTree>::New())
{
    Particle=new PARTICLE[n];
    for(int i=0;i<n;i++)
    {
        Particle[i].SetDim(dim);
    }
    PNum=n;
    GBestIndex=0;
    Xup=new double[dim];
    Xdown=new double[dim];
    Vmax=new double[dim];
//	W=1;
    W_max=1;
    W_min=0.6;

    C1=2;
    C2=2;
    Com=0;

    //将为每一轮点设置对应的PARTICLE，并为其设定初始矩阵
    double xup[3]{10.0,10.0,10.0};
    double xdown[3]{-10.0,-10.0,-10.0};
    SetXup(xup);		//设置微粒坐标上界
    SetXdown(xdown);		//设置微粒坐标下界
    SetVmax(0.5);		//设置微粒最大速度，以坐标的上下界的百分比为参数

}
void PSOICP::align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess)
{
    ToughMatirx = guess;
    //按选取的先后轮数分区，一个元素代表医生的一轮选点,存储到一个微粒当中
//    for(int i=0;i<6;i++)
//    {
//        vtkSmartPointer<vtkPoints> temp = vtkSmartPointer<vtkPoints>::New();
//        double temppoint[3];
//        for(int j=0;j<36;j=j+6)
//        {
//            cloud->GetPoint(j+i,temppoint);
//            temp->InsertNextPoint(temppoint);
//        }
//        Particle[i].Data->DeepCopy(temp);     //深拷贝，使得其可以循环使用
//    }
    PARTICLE result = Run(30);      //执行粒子群的优化算法，这里的run是初始化值 (这里返回了最佳的粒子，是6个粒子中的一个的引用，所以析构了这个粒子就也析构了6个粒子中的一个)

    Eigen::Matrix4f opt_tough;
    float x_angle = result.X[0]*M_PI/180.0;
    float y_angle = result.X[1]*M_PI/180.0;
    float z_angle = result.X[2]*M_PI/180.0;
    opt_tough = RotateZ(z_angle)*RotateY(y_angle)*RotateX(x_angle);
    opt_tough = opt_tough*ToughMatirx;

    MyICP  icp_1 ;
    icp_1.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
    icp_1.setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
    icp_1.setInputSource(input_);        //source经过变换到target,探针获取的特征点
    icp_1.setInputTarget(target_);        //target不变，CT模型
    vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
    icp_1.align(nonsense, opt_tough);

    EMS = icp_1.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
    finall_transform = icp_1.getFinalTransformation();

}
void PSOICP::Analysis()
{

}

void PSOICP::setCenterMatrix(const Eigen::Matrix4f& spacetoPlane)
{
     SpacetoPlane = spacetoPlane;
}
void PSOICP::setRangeofAngle(float angle)
{
    RangeofAngle = angle;
}

//针对该轮点，计算其最优的值
float PSOICP::GetFit(PARTICLE& OneParticle)
{
    static int count=0;
    count++;
    //std::cout<<"ICP"<<count<<"次"<<std::endl;
    //还原记录角度偏移的X数组，到旋转矩阵
    Eigen::Matrix4f x_matirx;
    float x_angle = OneParticle.X[0]*M_PI/180.0;
    float y_angle = OneParticle.X[1]*M_PI/180.0;
    float z_angle = OneParticle.X[2]*M_PI/180.0;
    x_matirx = RotateZ(z_angle)*RotateY(y_angle)*RotateX(x_angle);
    x_matirx = x_matirx*ToughMatirx;

    MyICP  icp_1 ;
    icp_1.setEuclideanFitnessEpsilon(0.00001);    //误差限，若超过该限制，则视为不收敛
    icp_1.setMaximumIterations(50);          //最大迭代次数，默认为100，这里设为300时收敛结果也一样
    icp_1.setInputSource(input_);        //source经过变换到target,探针获取的特征点
    icp_1.setInputTarget(target_);        //target不变，CT模型
    vtkSmartPointer<vtkPoints> nonsense = vtkSmartPointer<vtkPoints>::New();     //转换后的模型点云
    icp_1.align(nonsense, x_matirx);
    float sum_error = icp_1.getFitnessScore();   //计算对应点对之间的距离，这个这个要求set的Source和target的点云指针不能有更改，否则计算就不正确了
    return sum_error;
}


//微粒的无参构造函数
PARTICLE::PARTICLE()
{
    X=0;
    V=0;
    XBest=0;
    Dim=0;
}
//微粒的有参构造函数
PARTICLE::PARTICLE(int n)
{
    Dim=n;
    X=new double[Dim];
    V=new double[Dim];
    XBest=new double[Dim];
}
//微粒的析构函数
PARTICLE::~PARTICLE()
{
    if(Dim)
    {
        delete[] X;
        delete[] V;
        delete[] XBest;
        std::cout<<"delete one particle"<<std::endl;
    }
//    if(X)    //要确保手动删除的部分还存在，因为有可能浅拷贝出去之后，已经被析构掉了
//    {
//        delete[] X;
//        delete[] V;
//        delete[] XBest;
//       std::cout<<"delete one particle"<<std::endl;
//    }

}
//设置微粒的维数
void PARTICLE::SetDim(int d)
{
    if(X)
    {
        delete[] X;
    }
    if(V)
    {
        delete[] V;
    }
    if(XBest)
    {
        delete[] XBest;
    }
    Dim=d;
    X=new double[Dim];
    V=new double[Dim];
    XBest=new double[Dim];
}

//PSO的无参构造函数
PSO::PSO()
{
    Particle=0;
    PNum=0;
    GBestIndex=0;
    Xup=0;
    Xdown=0;
//	W=1;
    W_max=1;
    W_min=0.6;

    C1=2;
    C2=2;
    Com=0;
}
//PSO的有参构造函数
PSO::PSO(int dim,int n)
{
    Particle = new PARTICLE[n];
    for(int i=0;i<n;i++)
    {
        Particle[i].SetDim(dim);
    }

    PNum=n;
    GBestIndex=0;
    Xup=new double[dim];
    Xdown=new double[dim];
    Vmax=new double[dim];
//	W=1;
    W_max=1;
    W_min=0.6;

    C1=2;
    C2=2;
    Com=0;
}
//PSO的析构函数
PSO::~PSO()
{

    if(Xup)
    {
        delete[] Xup;
    }
    if(Xdown)
    {
        delete[] Xdown;
    }
    if(Vmax)
    {
        delete[] Vmax;
    }
    if(Particle)
    {
        delete[] Particle;
    }
}
//设置坐标上界
void PSO::SetXup(double *up)
{
    if(!Particle)
    {
        return;
    }
    for(int i=0;i<Particle[0].Dim;i++)
    {
        Xup[i]=up[i];
    }
}
//设置坐标下界
void PSO::SetXdown(double *d)
{
    if(!Particle)
    {
        return;
    }
    for(int i=0;i<Particle[0].Dim;i++)
    {
        Xdown[i]=d[i];
    }
}
//设置最大速度
void PSO::SetVmax(double *max)
{
    if(!Particle)
    {
        return;
    }
    for(int i=0;i<Particle[0].Dim;i++)
    {
        Vmax[i]=max[i];
    }
}
void PSO::SetVmax(double p)
{
    if(!Particle)
    {
        return;
    }
    for(int i=0;i<Particle[0].Dim;i++)
    {
        Vmax[i]=(Xup[i]-Xdown[i])*p;
    }
}
//初始化群体
void PSO::Initialize()
{
    if(!Particle)
    {
        return;
    }
    static int kk=(unsigned)time(NULL);
    srand((unsigned)time(NULL)+kk++);

    GBestIndex=0;

    //初始化所有粒子的个体
    for(int i=0;i<PNum;i++)
    {
        for(int j=0;j<Particle[i].Dim;j++)
        {
            Particle[i].X[j]=rand()/(double)RAND_MAX*(Xup[j]-Xdown[j])+Xdown[j];//随机初始化坐标
            Particle[i].XBest[j]=Particle[i].X[j];                     //将当前的最优矩阵值设置为初始化的值
            Particle[i].V[j]=rand()/(double)RAND_MAX*Vmax[j]-Vmax[j]/2;//随机初始化速度
        }
        Particle[i].Fit=GetFit(Particle[i]);//计算每个微粒适合度
        Particle[i].FitBest=Particle[i].Fit;//计算最优适合度值
        if(Particle[i].Fit<Particle[GBestIndex].Fit)
        {
            //如果这个鸟的适合度大于群体的最大适合度的话，记录下查找群体的最优微粒
            GBestIndex=i;
        }
    }
}
//计算群体各个微粒的适合度
void PSO::CalFit()
{
    if(!Particle)
    {
        return;
    }
    for(int i=0;i<PNum;i++)
    {
        Particle[i].Fit=GetFit(Particle[i]);
    }
}
//微粒飞翔，产生新一代微粒
void PSO::ParticleFly()
{
    static double FitBak[100];//用来存放备份的适合度值
    if(!Particle)
    {
        return;
    }
    static int tt=(unsigned)time(NULL);
    srand((unsigned)time(NULL)*tt++);

    static int kk=2;//迭代次数         --静态变量，函数退出时不会被回收
    double W;         //惯性系数
    W=W_max-kk*(W_max-W_min)/IteorMax;         //--惯性系数递减  从1递减到0.6    //还可以设置从1到0的随机周期下降，到0后再重置为1
    std::cout<<"最大迭代次数："<<IteorMax<<std::endl;
    kk++;

    //整个群体飞向新的位置
    for(int i=0;i<PNum;i++)
    {
        for(int j=0;j<Particle[i].Dim;j++)
        {
            //--局部最优和全局最优的系数 是随机变换的
            Particle[i].V[j]=W*Particle[i].V[j]+
                             rand()/(double)RAND_MAX*C1*(Particle[i].XBest[j]-Particle[i].X[j])+
                             rand()/(double)RAND_MAX*C2*(Particle[GBestIndex].XBest[j]-Particle[i].X[j]);

        }
        //--纠正速度
        for(int j=0;j<Particle[i].Dim;j++)
        {
            if(Particle[i].V[j]>Vmax[j])
            {
                Particle[i].V[j]=Vmax[j];
            }
            if(Particle[i].V[j]<-Vmax[j])
            {
                Particle[i].V[j]=-Vmax[j];
            }
        }
        //--应用速度，纠正坐标范围
        for(int j=0;j<Particle[i].Dim;j++)
        {
            Particle[i].X[j]+=Particle[i].V[j];//修改坐标
            if(Particle[i].X[j]>Xup[j])
            {
                Particle[i].X[j]=Xup[j];
            }
            if(Particle[i].X[j]<Xdown[j])
            {
                Particle[i].X[j]=Xdown[j];
            }
        }
    }

    //计算各微粒的适合度
    CalFit();
    for(int i=0;i<PNum;i++)
    {
        FitBak[i]=Particle[i].Fit;
    }
    //设置个体的最好位置
    for(int i=0;i<PNum;i++)
    {
        if(Particle[i].Fit<=Particle[i].FitBest)
        {
            Particle[i].FitBest=Particle[i].Fit;             //--更新最佳适应度
            for(int j=0;j<Particle[i].Dim;j++)
            {
                Particle[i].XBest[j]=Particle[i].X[j];    //--更新最佳坐标的维值
            }
        }
    }

    //设置群体中新的最优个体-------------------------------------------------------------------------------------用真正的全局最优作为替换！！！！！！！！！！！！！！！1
    GBestIndex=0;
    for(int i=0;i<PNum;i++)
    {
        if( (Particle[i].FitBest<=Particle[GBestIndex].FitBest) && i!=GBestIndex)
        {
            GBestIndex=i;
        }
    }
}
//按最多运行次数运行群粒算法，返回最优粒子
PARTICLE& PSO::Run(int n)
{
    IteorMax = n;
    Initialize();
    double *opt_p=new double[Particle[0].Dim];//通讯用数组，最优点坐标
    double **opt_a=new double*[PNum];		  //通讯用数组，所有点坐标

    for(int i=0;i<n;i++)
    {
        ParticleFly();
        std::cout<<"第"<<i<<"次飞行后的误差，"<<Particle[GBestIndex].FitBest<<std::endl;
        for(int k=0;k<Particle[0].Dim;k++)
        {
            opt_p[k]=Particle[GBestIndex].XBest[k];//拷贝当前最优点坐标
        }
        for(int k=0;k<PNum;k++)
        {
            opt_a[k]=Particle[k].X;//指向所有点坐标
        }
    }
    delete[] opt_p;
    delete[] opt_a;
    return Particle[GBestIndex];
}
//按最佳适合度运行群粒算法
PARTICLE& PSO::Run(double fit)
{
    Initialize();
    double *opt_p=new double[Particle[0].Dim];//通讯用数组，最优点坐标
    double **opt_a=new double*[PNum];		  //通讯用数组，所有点坐标

    //什么时候停止呢？
    do
    {
        ParticleFly();
        for(int k=0;k<Particle[0].Dim;k++)
        {
            opt_p[k]=Particle[GBestIndex].XBest[k];//拷贝最优点坐标
        }
        for(int k=0;k<PNum;k++)
        {
            opt_a[k]=Particle[k].X;//指向所有点坐标
        }
    }while(Particle[GBestIndex].FitBest<fit);
    delete[] opt_p;
    delete[] opt_a;
    return Particle[GBestIndex];
}
//返回最佳个体
double PSO::GetBest(double *r)
{
    for(int i=0;i<Particle[GBestIndex].Dim;i++)
    {
        r[i]=Particle[GBestIndex].XBest[i];
    }
    return Particle[GBestIndex].FitBest;
}

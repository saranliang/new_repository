#ifndef FITPLANE_H
#define FITPLANE_H
#include<string>
#include"Robost.h"
#include <vtkKdTree.h>
#include <vtkPoints.h>
#include <boost/shared_ptr.hpp>
#include"MathTool.h"
#include"myicp_vtk.h"
class CorrespondenceEstimationPTP:public CorrespondenceEstimationMY
{
    virtual void determineCorrespondences(Correspondences &correspondences, float& Error);
};
class FitPlane:public MyICP     //继承传统配准方法   --子类必须包含父类的头文件
{
    public:
    FitPlane()
    {	correspondence_estimation = new CorrespondenceEstimationPTP;}
    virtual void align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess); //执行配准变换后的源点云到Final,transform_2.matrix()为guess矩阵，即粗配准矩阵
    virtual void transform_es_svd_weight();
    virtual ~FitPlane(){}  //作为子类需要实现父类的纯虚函数
protected:
    float CountError();        //计算误差
};
void FitPlane::align(vtkSmartPointer<vtkPoints> cloud, const Eigen::Matrix4f& guess)
{
   if (isinit == false)      //初始化步骤，这里是初始化input_transformed变量
   {
       Init();
       isinit = true;
   }
   iteration_k++;
   finall_transform = guess;
   pre_transform = guess;      //保存前一步的变换矩阵
   EMS = 0.0;
   //初始化匹配点云
   //clock_t start;
   input_transformed->Reset(); //分配的空间不变(读取对应的索引位置还是之前的数)，但是看起来像是置空了，插入的话是从头插入
   vtk::transformation(input_, input_transformed, pre_transform);          //执行K-1的变换矩阵
   correspondence_estimation->setInputTarget(target_);							           //target 对应match
   correspondence_estimation->setInputSource(input_transformed);          //input 对应 query
   correspondence_->clear();
   correspondence_estimation->determineCorrespondences(*correspondence_,EMS);  //同时查找对应点、计算误差总和

  // std::cout << "初始误差: "<< EMS << std::endl;     //初始误差即为粗配准变换后的误差
   while (iteration_k < MaximumIteration_ && isConverged != true)
   {
       //根据当前对应点对计算最优变换try_transform,并叠加给finall_transform
       transform_es_svd_weight();              //

       if ((pre_transform - finall_transform).norm() / 16.0 < transformation_epsilon_)
       {
           isConverged = true;
       }
       //计算误差
       //应用转变矩阵,更新对应点
       input_transformed->Reset();
       vtk::transformation(input_, input_transformed, finall_transform);//将变换应用在点云中，得到新的Pnew
       correspondence_estimation->setInputSource(input_transformed);          //在判断收敛时已经更新
       correspondence_->clear();
      float preEms = 0.0;
       correspondence_estimation->determineCorrespondences(*correspondence_,preEms);  //计算新的对应点Qnew

       //通过误差阈值判断迭代是否收敛(旧误差-新误差)<?
       if (abs(EMS - preEms) < FitnessEpsilon_)
       {
           isConverged = true;
       }
       //更新EMS
       EMS = preEms;
       //将当前变换保留，即保留当前的对应关系
       pre_transform = finall_transform;

      // std::cout << "迭代次数为：" << iteration_k <<" 误差: "<<EMS<< std::endl;   //第21次时出现错误
       iteration_k++;
   }
   //std::cout << "总共迭代 了" << iteration_k << "次" << std::endl;
   cloud = input_transformed;     //将最优点云赋值到结果(深拷贝是将外界的值赋值给当前变量，而将局部指针赋值给当前变量时不需要使用深拷贝，因为局部指针直接被析构)
   //分析当前点簇集中的误差关系

}
//第6步 矩阵估计更新
void FitPlane::transform_es_svd_weight()
{

    int point_number = (*correspondence_).size();
    Eigen::MatrixXf  correspond_x(3, point_number);   //这里in扫描的点云：x,  out是模型的点云：y
    Eigen::MatrixXf  correspond_y(3, point_number);
    Eigen::VectorXf W = Eigen::VectorXf::Zero(correspond_x.cols());
    for (int i = 0;i < point_number;i++)
    {
        double* in = input_transformed->GetPoint((*correspondence_)[i].index_query);
        correspond_x.block(0, i, 3, 1)<<in[0],in[1],in[2];
        double* out = (*correspondence_)[i].match_point;   //将点云中的点转换为矩阵中的一列
        correspond_y.block(0, i, 3, 1)<<out[0],out[1],out[2];
    }

    W = (correspond_x - correspond_y).colwise().norm();//2阶范式，每个点的距离
    robust_weight(Para.f, W, Para.p,SetSize);          //根据设定的鲁棒函数Para.f和鲁棒参数Para.p计算权值W
    /// Normalize weight vector
    Eigen::VectorXf w_normalized = W / W.sum();        //归一化，所有点的权值之和为1
    //std::cout<<"权值为"<<w_normalized<<std::endl;
    /// De-mean
    Eigen::Vector3f meanVec_x, meanVec_y;
    Eigen::MatrixXf zeroMeanMat_x = correspond_x;
    Eigen::MatrixXf zeroMeanMat_y = correspond_y;
    for (int i = 0; i<3; ++i) {
        meanVec_x(i) = (correspond_x.row(i).array()*w_normalized.transpose().array()).sum();
        meanVec_y(i) = (correspond_y.row(i).array()*w_normalized.transpose().array()).sum();
    }
    zeroMeanMat_x.colwise() -= meanVec_x;
    zeroMeanMat_y.colwise() -= meanVec_y;


    //计算协方差，行数为数据个数，列数为数据维数 这样的数据计算时是 A.T*A
    //但是在矩阵运算时，点的存储形式往往是3*1,所以是A*A.T
    Eigen::MatrixXf covMat = zeroMeanMat_x*w_normalized.asDiagonal() *zeroMeanMat_y.adjoint();

    //SVD分解求出旋转和平移矩阵，结果的事 y = R*x +T ，即在构建数据时做的变换的逆变换 = 金标准
    //ComputeThinU是计算非方阵的矩阵U，ComputeFullU计算的U为方阵
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(covMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //自己的方法
    Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
    float det = (U*V.transpose()).determinant();  //求行列式
    Eigen::Matrix3f M;
    M << 1, 0, 0, 0, 1, 0, 0, 0, det;
    Eigen::Matrix3f R = V*M*U.transpose();
    Eigen::MatrixXf T = meanVec_y - R*meanVec_x;
    try_transform.setZero();        //初始化为0矩阵
    try_transform.block(0, 0, 3, 3) = R;
    try_transform.block(0, 3, 3, 1) = T;
    try_transform(3, 3) = 1;
    finall_transform = try_transform*finall_transform;
}

void CorrespondenceEstimationPTP::determineCorrespondences(Correspondences &correspondences, float& Error)
{
    int source_number = input_->GetNumberOfPoints();
     correspondences.resize(source_number);
    unsigned int nr_valid_correspondences = 0;
    Correspondence corr;

     // Find the k closest points to testPoint
     unsigned int k = 6;   //待查找临近点的个数
     double *testPoint;    //待查找点
    // vtkNew<vtkIdList> result;   //error: no viable conversion from 'vtkNew<vtkIdList>' to 'vtkIdList *'
     vtkSmartPointer<vtkIdList> index = vtkSmartPointer<vtkIdList>::New();
     for(int i=0;i<source_number;i++)
     {
         testPoint = input_->GetPoint(i);
         tree_->FindClosestNPoints(k, testPoint, index);

         //
         vtkIdType point_ind = index->GetId(0);
         double *goal = target_->GetPoint(point_ind);

         //拟合曲面或直面
         vtkSmartPointer<vtkPoints> near_point = vtkSmartPointer<vtkPoints>::New();
         for(size_t i =0;i<k;i++)
         {
             vtkIdType point_ind = index->GetId(i);
             near_point->InsertNextPoint(target_->GetPoint(point_ind));
              //std::cout<<near_point->GetPoint(i)[0]<<" "<<near_point->GetPoint(i)[1]<<" "<<near_point->GetPoint(i)[2]<<std::endl;
         }
         Eigen::Vector3f Para = FitPlane(near_point);


         float A =Para(0);
         float B =Para(1);
         float C = Para(2);

         float distance  = (A*testPoint[0]+B*testPoint[1]+C*testPoint[2]+1.0)/sqrt(A*A+B*B+C*C);
         //std::cout<<"距离为"<<distance<<std::endl;

          double closed[3];
          closed[0] = -1.0*A*distance+testPoint[0];
          closed[1] = -1.0*B*distance+testPoint[1];
          closed[2] = -1.0*C*distance+testPoint[2];

        // std::cout<<closed[0]<<" "<<closed[1]<<" "<<closed[2]<<std::endl;
        // std::cout<<"------------"<<std::endl;

         corr.index_query = i;
         memcpy(corr.match_point, closed, 3 * sizeof(float));

         corr.distance = std::sqrt(std::pow((testPoint[0]-goal[0]),2)+std::pow((testPoint[1]-goal[1]),2)+std::pow((testPoint[2]-goal[2]),2));
         correspondences[nr_valid_correspondences++] = corr;

         Error+= corr.distance;    //存储误差
     }
}
#endif // FITPLANE_H

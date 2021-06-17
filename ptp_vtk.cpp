#include"ptp_vtk.h"
#include <vtkPolyDataNormals.h> //计算法向量
#include <vtkPointData.h>
#include<vtkFloatArray.h>
#include <vtkDoubleArray.h>
//法向量赋值:利用stl的三角网格计算
void PTPlaneICP::setNormals(vtkSmartPointer<vtkPolyData> STLCloud)
{
    //法向量计算
    vtkSmartPointer<vtkPolyDataNormals> normFilter =
            vtkSmartPointer<vtkPolyDataNormals>::New();

    normFilter->SetInputData(STLCloud); //需要输入带点所在两条边的信息的数据，因此不能仅仅包含points,还要有三角网格信息
    normFilter->SetComputePointNormals(1);//开启点法向量计算
    normFilter->SetComputeCellNormals(0); //关闭单元法向量计算
    normFilter->SetAutoOrientNormals(1);
    normFilter->SetSplitting(0);                            //关闭锐边缘处理，保证计算前后模型数据不会改变
    normFilter->Update();

    target_normals = normFilter->GetOutput();  //此时的计算时正常的
    auto normArray = dynamic_cast<vtkFloatArray*>(target_normals->GetPointData()->GetNormals());   //  ------计算后的法向量
    //target_normals->GetPoints()->GetPoint(i,temp_normals)   ---------法向量对应的点云中的点坐标
    double temp_normals[3];
    for(int i=0;i<target_normals->GetPoints()->GetNumberOfPoints();i++)
    {
        normArray->GetTuple(i,temp_normals);
    }
}
void PTPlaneICP::transform_es_svd_weight()
{
    //读取target对应的法向量
    vtkSmartPointer<vtkPolyData> y_normal = vtkSmartPointer<vtkPolyData>::New();
    auto target_NormalsArray = dynamic_cast<vtkFloatArray*>(target_normals->GetPointData()->GetNormals());
    vtkSmartPointer<vtkPoints> corr_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkFloatArray> corr_NormalsArray = vtkSmartPointer<vtkFloatArray>::New();
    corr_NormalsArray->SetNumberOfComponents(3); // 3d normals (ie x,y,z)
    corr_NormalsArray->SetNumberOfTuples(M);
    double temp_point[3];   //这里必须要是double型的数组
    double temp_normal[3];   //这里必须要是double型的数组
    for (int i = 0;i < M;i++)
    {
        int corr_index = (*correspondence_)[i].index_match;
        target_normals->GetPoints()->GetPoint(corr_index,temp_point);
        corr_points->InsertNextPoint(temp_point);

        target_NormalsArray->GetTuple(corr_index,temp_normal);
        corr_NormalsArray->SetTuple(i,temp_normal);    //因为提前分配好了空间，所以使用setTuple就可以
    }
    y_normal->SetPoints(corr_points);
    y_normal->GetPointData()->SetNormals(corr_NormalsArray);
    auto normArray = dynamic_cast<vtkFloatArray*>(y_normal->GetPointData()->GetNormals());
    double temp_normals[3];
    for(int i=0;i<y_normal->GetPoints()->GetNumberOfPoints();i++)
    {
        normArray->GetTuple(i,temp_normals);
    }
  //计算当前对应点对的权值
    Eigen::MatrixXf  correspond_x(3, M);   //这里in扫描的点云：x,  out是模型的点云：y
    Eigen::MatrixXf  correspond_y(3, M);
    Eigen::VectorXf W = Eigen::VectorXf::Zero(correspond_x.cols());
    for (int i = 0;i < M;i++)
    {
        double* in =   input_transformed->GetPoint((*correspondence_)[i].index_query);
        double* out = corr_points->GetPoint(i);   //将点云中的点转换为矩阵中的一列
        correspond_x.block(0, i, 3, 1) <<in[0],in[1],in[2];       //将点云中的点转换为矩阵中的一列
        correspond_y.block(0, i, 3, 1) <<out[0],out[1],out[2];
    }
    //计算权值
    W = (correspond_x - correspond_y).colwise().norm();//2阶范式，每个点的距离

//    std::cout<<"误差权重"<<std::endl;
//    std::cout<<W<<std::endl;

    //设置鲁棒权重
    robust_weight(Para.f, W, Para.p,SetSize);
    Eigen::VectorXf w_normalized = W / W.sum();        //归一化，所有点的权值之和为1

    //将Eigen::vector转化为std::vector
    std::vector<float>vec_weight (w_normalized.data(),w_normalized.data()+M);               //将Eigen中的向量元素转化为c++中的vector

    //最小化的是距离误差的平方和，因此实际上法向量是否相反不影响配准的结果。需要设计出对法向量敏感的距离和对应匹配方法
    //点到平面距离的线性优化
    //pcl::registration::TransformationEstimationPointToPlaneLLS < pcl::PointXYZ, pcl::PointNormal> ::Ptr  PTPLLS(new pcl::registration::TransformationEstimationPointToPlaneLLS < pcl::PointXYZ, pcl::PointNormal>);
    //点到平面距离的非线性优化
    //pcl::registration::TransformationEstimationPointToPlane < pcl::PointXYZ, pcl::PointNormal> ::Ptr  PTPLLS(new pcl::registration::TransformationEstimationPointToPlane < pcl::PointXYZ, pcl::PointNormal>);
    //点到平面的带权值优化
    //pcl::registration::TransformationEstimationPointToPlaneLLSWeighted < pcl::PointXYZ, pcl::PointNormal>::Ptr PTPLLS(new pcl::registration::TransformationEstimationPointToPlaneLLSWeighted < pcl::PointXYZ, pcl::PointNormal> );
    //PTPLLS->setCorrespondenceWeights(vec_weight);

    //PTPLLS->estimateRigidTransformation(*input_transformed, *y_normal, try_transform);           //这里的target要带有法向量


    //自己拷贝重写的方法
    estimateRigidTransformation_ptp(input_transformed, y_normal, vec_weight.begin(),try_transform);

    finall_transform = try_transform*finall_transform;


   //点到直线的矩阵估计
   //pcl::registration::TransformationEstimationPointToLine<pcl::PointXYZ,pcl::PointXYZ>::Ptr  PLTF(new pcl::registration::TransformationEstimationPointToLine< pcl::PointXYZ, pcl::PointNormal>);
   //PLTF->setTargetTree(*target_);
    //PLTF->estimateRigidTransformation(*input_transformed, *target_, try_transform);           //这里的target要带有法向量
}
void estimateRigidTransformation_ptp(const vtkSmartPointer<vtkPoints> source,const vtkSmartPointer<vtkPolyData> target,std::vector<float>::iterator weights_it,Eigen::Matrix4f& transformation_matrix)
{
      typedef Eigen::Matrix<double, 6, 1> Vector6d;
      typedef Eigen::Matrix<double, 6, 6> Matrix6d;

      Matrix6d ATA;
      Vector6d ATb;
      ATA.setZero ();
      ATb.setZero ();

      int point_number = source->GetNumberOfPoints();
      double source_point[3];
      double target_point[3];
      double target_normal[3];
      auto normArray = dynamic_cast<vtkFloatArray*>( target->GetPointData()->GetNormals());
      for(int i=0;i<point_number;i++)
      {
          source->GetPoint(i,source_point);
          target->GetPoint(i,target_point);
          normArray->GetTuple(i,target_normal);
          if (!std::isfinite (source_point[0]) ||
              !std::isfinite (source_point[1]) ||
              !std::isfinite (source_point[2]) ||
              !std::isfinite (target_point[0]) ||
              !std::isfinite (target_point[1]) ||
              !std::isfinite (target_point[2]) ||
              !std::isfinite (target_normal[0]) ||
              !std::isfinite (target_normal[1]) ||
              !std::isfinite (target_normal[2]))
          {
             ++weights_it;
            continue;
          }

        const double & sx = source_point[0];
        const double & sy = source_point[1];
        const double & sz = source_point[2];
        const double & dx = target_point[0];
        const double & dy = target_point[1];
        const double & dz = target_point[2];
        const double & nx = target_normal[0] * static_cast<double>(*weights_it);
        const double & ny = target_normal[1] * static_cast<double>(*weights_it);
        const double & nz = target_normal[2] * static_cast<double>(*weights_it);

        double a = nz*sy - ny*sz;
        double b = nx*sz - nz*sx;
        double c = ny*sx - nx*sy;

        //    0  1  2  3  4  5
        //    6  7  8  9 10 11
        //   12 13 14 15 16 17
        //   18 19 20 21 22 23
        //   24 25 26 27 28 29
        //   30 31 32 33 34 35

        ATA.coeffRef (0) += a * a;
        ATA.coeffRef (1) += a * b;
        ATA.coeffRef (2) += a * c;
        ATA.coeffRef (3) += a * nx;
        ATA.coeffRef (4) += a * ny;
        ATA.coeffRef (5) += a * nz;
        ATA.coeffRef (7) += b * b;
        ATA.coeffRef (8) += b * c;
        ATA.coeffRef (9) += b * nx;
        ATA.coeffRef (10) += b * ny;
        ATA.coeffRef (11) += b * nz;
        ATA.coeffRef (14) += c * c;
        ATA.coeffRef (15) += c * nx;
        ATA.coeffRef (16) += c * ny;
        ATA.coeffRef (17) += c * nz;
        ATA.coeffRef (21) += nx * nx;
        ATA.coeffRef (22) += nx * ny;
        ATA.coeffRef (23) += nx * nz;
        ATA.coeffRef (28) += ny * ny;
        ATA.coeffRef (29) += ny * nz;
        ATA.coeffRef (35) += nz * nz;

        double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
        ATb.coeffRef (0) += a * d;
        ATb.coeffRef (1) += b * d;
        ATb.coeffRef (2) += c * d;
        ATb.coeffRef (3) += nx * d;
        ATb.coeffRef (4) += ny * d;
        ATb.coeffRef (5) += nz * d;
        ++weights_it;
      }

      ATA.coeffRef (6) = ATA.coeff (1);
      ATA.coeffRef (12) = ATA.coeff (2);
      ATA.coeffRef (13) = ATA.coeff (8);
      ATA.coeffRef (18) = ATA.coeff (3);
      ATA.coeffRef (19) = ATA.coeff (9);
      ATA.coeffRef (20) = ATA.coeff (15);
      ATA.coeffRef (24) = ATA.coeff (4);
      ATA.coeffRef (25) = ATA.coeff (10);
      ATA.coeffRef (26) = ATA.coeff (16);
      ATA.coeffRef (27) = ATA.coeff (22);
      ATA.coeffRef (30) = ATA.coeff (5);
      ATA.coeffRef (31) = ATA.coeff (11);
      ATA.coeffRef (32) = ATA.coeff (17);
      ATA.coeffRef (33) = ATA.coeff (23);
      ATA.coeffRef (34) = ATA.coeff (29);

      // Solve A*x = b

      Vector6d x = static_cast<Vector6d> (ATA.inverse () * ATb);   //行列式不为0时，表示可逆


      double alpha = x (0);
      double beta = x(1);
      double gamma = x(2);

      double tx =x(3);
      double ty =x(4);
      double tz =x(5);
      // Construct the transformation matrix from rotation and translation
      transformation_matrix = Eigen::Matrix<float, 4, 4>::Zero ();

      transformation_matrix (0, 0) = static_cast<float> ( cos (gamma) * cos (beta));
      transformation_matrix (0, 1) = static_cast<float> (-sin (gamma) * cos (alpha) + cos (gamma) * sin (beta) * sin (alpha));
      transformation_matrix (0, 2) = static_cast<float> ( sin (gamma) * sin (alpha) + cos (gamma) * sin (beta) * cos (alpha));
      transformation_matrix (1, 0) = static_cast<float> ( sin (gamma) * cos (beta));
      transformation_matrix (1, 1) = static_cast<float> ( cos (gamma) * cos (alpha) + sin (gamma) * sin (beta) * sin (alpha));
      transformation_matrix (1, 2) = static_cast<float> (-cos (gamma) * sin (alpha) + sin (gamma) * sin (beta) * cos (alpha));
      transformation_matrix (2, 0) = static_cast<float> (-sin (beta));
      transformation_matrix (2, 1) = static_cast<float> ( cos (beta) * sin (alpha));
      transformation_matrix (2, 2) = static_cast<float> ( cos (beta) * cos (alpha));

      transformation_matrix (0, 3) = static_cast<float> (tx);
      transformation_matrix (1, 3) = static_cast<float> (ty);
      transformation_matrix (2, 3) = static_cast<float> (tz);
      transformation_matrix (3, 3) = static_cast<float> (1);
}

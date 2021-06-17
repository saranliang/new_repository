#ifndef MATHTOOL_H    //防止在不同的头文件中进行两段相同的定义
#define MATHTOOL_H
#include<vector>
#include <iostream>
#include<string>
#include <math.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include<vtkPolyDataWriter.h>
#include<vtkPolyDataReader.h>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <string.h>
#define BOOST_TYPEOF_EMULATION
#define _USE_MATH_DEFINES       //宏定义，用来与之后的代买进行交互，define

namespace vtk                    //在相同的命名空间下，不能定义同名的两个类模板
{
    template <typename Scalar = float>   //这个cloud_out需要是个空的集合，可以从头插入
    //cloud_in :不会改变            cloud_out:旋转后的点云
    void transformation(vtkSmartPointer<vtkPoints> cloud_in, vtkSmartPointer<vtkPoints> cloud_out, const Eigen::Matrix<Scalar,4,4> &rotation)
    {
       int point_number =cloud_in->GetNumberOfPoints();
       for(int i=0;i<point_number;i++)
       {
           Eigen::Matrix<Scalar, 3, 1> pt (cloud_in->GetPoint(i)[0], cloud_in->GetPoint(i)[1], cloud_in->GetPoint(i)[2]);
           Scalar out[3];
           out[0] = static_cast<Scalar> (rotation (0, 0) * pt.coeffRef (0) + rotation (0, 1) * pt.coeffRef (1) + rotation (0, 2) * pt.coeffRef (2) + rotation (0, 3));
           out[1] = static_cast<Scalar> (rotation (1, 0) * pt.coeffRef (0) + rotation (1, 1) * pt.coeffRef (1) + rotation (1, 2) * pt.coeffRef (2) + rotation (1, 3));
           out[2] = static_cast<Scalar> (rotation (2, 0) * pt.coeffRef (0) + rotation (2, 1) * pt.coeffRef (1) + rotation (2, 2) * pt.coeffRef (2) + rotation (2, 3));
           cloud_out->InsertNextPoint(out);
       }
    }
}


//数字全排列
//https://leetcode-cn.com/problems/permutations/solution/quan-pai-lie-by-leetcode-solution-2/
class Permutation {
public:
    void backtrack(std::vector<std::vector<int>>& res, std::vector<int>& output, int first, int len) {    //写在类内部的定义视为inline,因此放在.h文件中不会发生重定义
        // 所有数都填完了
        if (first == len) {
            res.emplace_back(output);  //在容器尾部添加一个元素，这个元素原地构造，不需要触发拷贝构造和转移构造
            return;
        }
        //遍历待插入数组中的所有数
        for (int i = first; i < len; ++i) {
            // 动态维护数组
            std::swap(output[i], output[first]);
            // 继续递归填下一个数
            backtrack(res, output, first + 1, len);
            // 撤销操作
            std::swap(output[i], output[first]);
        }
    }
    std::vector<std::vector<int>> permute(std::vector<int>& nums) {
        std::vector<std::vector<int> > res;
        backtrack(res, nums, 0, (int)nums.size());
        return res;
    }
};
//求由旋转平移的构建的齐次矩阵的逆矩阵
Eigen::Matrix4f HomoInverse(const Eigen::Matrix4f& Source);

//拟合空间平面
//https://blog.csdn.net/konglingshneg/article/details/82585868
//克拉默法则行列式求解 ：https://my.oschina.net/u/4266515/blog/3329781
//最小二乘解：https://blog.csdn.net/qq_40335930/article/details/100546496?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~aggregatepage~first_rank_v2~rank_aggregation-7-100546496.pc_agg_rank_aggregation&utm_term=c%2B%2B+%E6%8B%9F%E5%90%88%E5%B9%B3%E9%9D%A2&spm=1000.2123.3001.4430
//SVD分解：    https://blog.csdn.net/qq_37569355/article/details/112620901?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~aggregatepage~first_rank_v2~rank_aggregation-2-112620901.pc_agg_rank_aggregation&utm_term=c%2B%2B+%E6%8B%9F%E5%90%88%E5%B9%B3%E9%9D%A2&spm=1000.2123.3001.4430
// Para:A,B,C,D       Ax+By+Cz+1=0
Eigen::MatrixXf FitPlane(vtkSmartPointer<vtkPoints> fit_cloud);
//点到平面的投影 target = source-k*normal
//@Para :平面法向量参数
//@source:待投影空间点
//@测试投影后的点是否真的在平面上：OK
Eigen::Vector3f ProjectiToPlane(const Eigen::Vector3f& Para,const Eigen::Vector3f& source);
//从空间坐标转换到平面坐标
//@PlanetoSpace： 空间坐标 = PlanetoSpace*平面坐标
//@origin:人为设定的转换后的坐标原点,默认为原来的原点投影点
void  PlaneCoordinate(const Eigen::Vector3f& Para,Eigen::Matrix4f& PlanetoSpace, Eigen::Vector3f Origin);
//平面上拟合圆
//@fit_points:待拟合点集（点维数*点个数）    =向量都是作为一列来存储的
//@Para:(圆心x坐标，圆心y坐标，半径)
void FitCircle(const Eigen::MatrixXf& fit_points,  Eigen::Vector3f& Para);
//根据空间点拟合对应圆的圆心及对应的位姿变换矩阵
Eigen::MatrixXf FitCircle3D(const vtkSmartPointer<vtkPoints> fit_cloud,  Eigen::Vector3f& Origin);

//旋转平移矩阵定义------------------------------------------------------------------------------------------------
//运算过程都是齐次坐标
//连乘矩阵可以任意加括号
//绕X轴正向旋转Angle角度的旋转矩阵
Eigen::Matrix4f RotateX(float Angle);
//绕Y轴正向旋转
Eigen::Matrix4f RotateY(float Angle);
//绕Z轴正向旋转
Eigen::Matrix4f RotateZ(float Angle);

//四元数转欧拉角(自定义的方法，避免二义性的出现，在左手坐标系下适用)
void toEulerAngle(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw);

//平移矩阵
Eigen::Matrix4f Translation(float X, float Y, float Z);
//从四元数到旋转矩阵
Eigen::Matrix4f Quaternion2RotationMatrix(const float x, const float y, const float z, const float w);
//从旋转矩阵到四元数
Eigen::Quaternionf RotationMatrix2Quaternion(Eigen::Matrix3f R);

bool isSamePoint(float a[3],float b[3]);
void doubletofloat(double a[3],float b[3]);

void savaPointsVTK(vtkSmartPointer<vtkPoints> cloud,const char* name);
void loadPointsVTK(vtkSmartPointer<vtkPoints>& cloud,const char* name);
#endif
#pragma once

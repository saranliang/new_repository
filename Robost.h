#ifndef ROBOST_H
#define ROBOST_H
#include<iostream>
#include<vector>
#include<string>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
//设置不同的鲁棒估计函数
enum Function {
    PNORM,
    TUKEY,
    DIS_TUKEY,
    FAIR,
    LOGISTIC,
    TRIMMED,
    HUBER,
    MEDIAN,
    DIS_TRIMMED,
    DIS_PNORM,
    TANH,
    SET,
    NONE
};

//使用哈希值来实现对于字符串的switch
//https://blog.csdn.net/yozidream/article/details/22789147
typedef std::uint64_t hash_t;
 constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;

 //将字符串转为一个整数
 hash_t hash_(char const* str);
//编译时将字符串计算为哈希常量
constexpr hash_t hash_compile_time(char const* str, hash_t last_value);
//定义运算符重载
constexpr unsigned long long operator "" _hash(char const* p, size_t);

class Parameters {
public:
    Parameters(Function robost = NONE, float p = 0.1)   //默认的构造函数时没有误差权值的
	{
        this->f = robost;
		this->p = p;
	}
    Parameters(std::string RobostName,float p);
	/// Parameters
	Function f;     /// robust function type
	float p;       /// paramter of the robust function   
};
struct sort_pred {
    bool operator()(const std::pair<int, float> &left,
        const std::pair<int, float> &right) {
        return left.second < right.second;
    }
};
//平均权值
extern void uniform_weight(Eigen::VectorXf& r);
//P阶范式权值，误差越大，权值越小；P值越大，y'=a*x^(a-1)，y'=a^x*lna
//p=1时，约等于距离的倒数

extern void pnorm_weight(Eigen::VectorXf& r, float p, float reg );

//部分截断，部分降低
extern void tukey_weight(Eigen::VectorXf& r, float p);

extern void fair_weight(Eigen::VectorXf& r, float p);

extern void logistic_weight(Eigen::VectorXf& r, float p);

//截断法，将所有点对按照配准距离排序，选取前百分之P的点对权值为1，剩余为0
//修改之后r代表的是权值，不再是距离！！！！这里有问题
extern void trimmed_weight(Eigen::VectorXf& r, float p);

extern void huber_weight(Eigen::VectorXf& r, float p);
extern void median_wegiht(Eigen::VectorXf& r, float p);

//反向P阶，误差越大，权值越大
extern void  dis_pnorm_weight(Eigen::VectorXf& r, float p, float reg);
//激活函数的一种，值域为-1，1，奇函数，单调递增
extern void tanh_weight(Eigen::VectorXf& r, float p);
extern void set_weight(Eigen::VectorXf& r, float p,const std::vector<int>& setSize);
extern void robust_weight(Function f, Eigen::VectorXf& r, float p,const std::vector<int>& setSize);

#endif
#pragma once

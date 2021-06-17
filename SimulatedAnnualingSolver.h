#ifndef SIMULATEDANNUALINGSOLVER_H
#define SIMULATEDANNUALINGSOLVER_H

#include<random>

class SimulatedAnnualingSolver
{
public:
	SimulatedAnnualingSolver(int rand_seed);
	void setParameters(double t_initial, double mu_t, double k = 1, double t_min = 1e-5, int iters_fixed_T = 100, double step_size = 1);
	void enablePrint(bool on) { m_print_position = on; }
	void solve();
protected:
	virtual double take_step(double step_size) = 0;    //用来在解空间中游走到一个新的状态
	virtual void undo_step() = 0;                    //撤销最近的一次游走
	virtual void save_best() = 0;                    //保存当前的状态，作为当前最佳解
	virtual double energy() = 0;                     //目标函数
	virtual void print() = 0;                        //打印当前状态
	virtual void SetBest() = 0;

    int m_iters_fixed_T;                             //内循环迭代步数
private:
	bool m_print_position;	
	double m_step_size;                              //迭代步长
	double m_k;                                     //波兹曼常数，默认为1
	double m_t_initial;                              //最初温度
	double m_mu_t;                                  //1/m_mu_t 为温度冷却因子
	double m_t_min;                                 //最低温度
	std::mt19937 m_randGenerator;
	double boltzmann(double E, double new_E, double T, double k);
protected:                                          //设为保护变量，才能被子类访问
	double T;                                       //实时温度
};

#endif // SIMULATEDANNUALINGSOLVER_H
#pragma once

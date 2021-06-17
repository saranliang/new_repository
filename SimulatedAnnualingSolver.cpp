#include "SimulatedAnnualingSolver.h"
#include <cmath>
#include <iostream>
#define GSL_LOG_DBL_MIN   (-7.0839641853226408e+02)

SimulatedAnnualingSolver::SimulatedAnnualingSolver(int rand_seed)          //1、用流逝的时间得到一个种子 unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count()
	:m_t_initial(1),
    m_mu_t(1.2),                           //初始为1.01
	m_k(1),
    m_t_min(0.01),                         //初始为0.01
    m_iters_fixed_T(30),                    //初始为100
	m_print_position(false),
	m_step_size(1),
	m_randGenerator(rand_seed)                                         //2、用种子构建一个随机生成器
{

}

void SimulatedAnnualingSolver::setParameters(double t_initial, double mu_t, double k, double t_min, int iters_fixed_T, double step_size)
{
	m_t_initial = t_initial;
	m_mu_t = mu_t;
	m_k = k;
	m_t_min = t_min;
	m_iters_fixed_T = iters_fixed_T;
	m_step_size = step_size;
}

void SimulatedAnnualingSolver::solve()
{
	int n_evals = 1, n_iter = 0;
	double E = energy();                        //E为前一步的解
	if (E < 1)                                 //若最初的误差已经小于1了，则无需优化
	{
		return;
	}
	double best_E = E;                          //best_E为最优解
	save_best(); // 将当前状态存为最佳解
	
	std::cout << "误差为" << E << std::endl;

	T = m_t_initial;
	double T_factor = 1.0 / m_mu_t;
	std::uniform_real_distribution<> dis(0, 1);                           //3、定义一个浮点均匀分布函数
    int no_best = 0;
	while (1)
	{
		int n_accepts = 0;     //接受较差解次数
		int n_rejects = 0;     //拒绝较差解次数
		int n_eless = 0;       //得到较好解次数

		for (int i = 0; i < m_iters_fixed_T; ++i)
		{
            double new_E = take_step(i); //更新参数矩阵
            //std::cout<< i<<" "<<new_E<<std::endl;
			if (new_E <= best_E)                 //若小于最优解，则直接保存当前参数矩阵为最优矩阵，并更新最优解
			{
				best_E = new_E;
				save_best();
                no_best = 0;
			}

			++n_evals;
			if (new_E < E)                      //若小于前驱解，则直接接受为前驱解，但并不保存参数矩阵
			{
				/*if (new_E < best_E)
				{
					best_E = new_E;
					save_best();
				}*/
				/* yay! take a step */
				E = new_E;
				++n_eless;
			}
			//以一定的概率来接受一个比当前解要差的解
            else if (dis(m_randGenerator) < boltzmann(E, new_E, T, m_k)) //4、传入随机生成器，调用范围分布对象
            {
                /* yay! take a step */
                E = new_E;
                ++n_accepts;
                std::cout<<"接受了差解"<<std::endl;
            }
			else
			{
				undo_step(); // 不接受时，回退到上一个状态
				++n_rejects;
			}
		}

        no_best++;

        printf("%5d   %7d  %12g", n_iter, n_evals, T);
        print();
        printf("  %12g  %12g\n", E, best_E);
        std::cout<<std::endl;
		/*std::cout << "接受差解" << n_accepts << "次" << std::endl;
		std::cout << "拒绝差解" << n_rejects << "次" << std::endl;
		std::cout << "得到好解" << n_eless << "次" << std::endl;
		std::cout << "温度为" << T << std::endl;
		std::cout << "误差为" << E << std::endl;*/

		/* apply the cooling schedule to the temperature */
		T *= T_factor;      //冷却温度
		++n_iter;
        if (T < m_t_min||no_best>5)    //温度低于设定值时
		{
			std::cout << "迭代次数为" << n_iter << std::endl;
			std::cout << "最优误差为" << best_E << std::endl;
			SetBest();
			break;
		}
	}
}

//计算接收较差解的概率，温度越大，接受坏解的概率越大
inline double SimulatedAnnualingSolver::boltzmann(double E, double new_E, double T, double k)
{
	double x = -(new_E - E) / (k * T);
	/* avoid underflow errors for large uphill steps */
	return (x < GSL_LOG_DBL_MIN) ? 0.0 : exp(x);
}

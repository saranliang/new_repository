#include "ExternalPoleLine.h"
#define R_Range 0.05
#define T_Range 0.5
SA::SA(int rand_seed, SAICP *FirstGT)
	:SimulatedAnnualingSolver(rand_seed)
{
	std::random_device r;
	std::seed_seq seed2{ r(), r(), r(), r(), r(), r(), r(), r() };
	m_randGen.seed(seed2);                                      //用种子构建随机生成器
	Optimal_gt = FirstGT;
	GetRang();
}

void SA::save_best()                                     //最优解获得更新时，保存当前矩阵
{
	for (int i = 0;i < 7;i++)
	{
		BestGtArray[i] = Optimal_gt->gt_array[i];
	}
}
void SA::SetBest()
{
	for (int i = 0;i < 7;i++)
	{
		Optimal_gt->gt_array[i] = BestGtArray[i];
	}
	Optimal_gt->updateRT();
}

void SA::print()
{
	
}

double SA::take_step(double step_size)                       //更新矩阵，将更新前的矩阵保存在PreGtArray
{
	double max_parament, min_parament;
	double temp_parament;
	for (int i = 0;i < 7;i++)
	{
		max_parament = MaxParament[i];
		min_parament = MinParament[i];
		PreGtArray[i] = Optimal_gt->gt_array[i];
		Cauchy(max_parament, min_parament, Optimal_gt->gt_array[i]);
	}
	double result = energy();
	return result;
}

void SA::undo_step()                                       //回退到上一个状态，即读取前驱矩阵
{
	Optimal_gt->gt_array[0] = PreGtArray[0];
	Optimal_gt->gt_array[1] = PreGtArray[1];
	Optimal_gt->gt_array[2] = PreGtArray[2];
	Optimal_gt->gt_array[3] = PreGtArray[3];
	Optimal_gt->gt_array[4] = PreGtArray[4];
	Optimal_gt->gt_array[5] = PreGtArray[5];
	Optimal_gt->gt_array[6] = PreGtArray[6];

	Optimal_gt->updateRT();
}

double SA::energy()
{
	Eigen::VectorXd GT(7);
	GT(0) = Optimal_gt->gt_array[0];
	GT(1) = Optimal_gt->gt_array[1];
	GT(2) = Optimal_gt->gt_array[2];
	GT(3) = Optimal_gt->gt_array[3];
	GT(4) = Optimal_gt->gt_array[4];
	GT(5) = Optimal_gt->gt_array[5];
	GT(6) = Optimal_gt->gt_array[6];
	double fvec = Optimal_gt->fx(GT);
	return fvec;
}
//也是生成随机数，再按照设定的函数得出震荡后的值，因此也有可能重复
void SA::Cauchy(double max, double min, double& temp_m)
{
	double result_m = 0;
	while(1)
	{
		std::uniform_real_distribution<> dis(0, 1);
		double u = dis(m_randGen);
		double yi = T*sgn(u - 0.5)*(pow(1 + 1 / T, abs(2 * u - 1)) - 1);
		result_m = temp_m + (max - min)*yi;
		if (result_m > min&&result_m < max)                  //要保证新生成的参数也在取值范围内
		{
			break;
		}
	}
	temp_m = result_m;
}
void SA::GetRang()
{
	for (int i = 0;i < 3;i++)
	{
		MaxParament[i] = Optimal_gt->gt_array[i] + R_Range;
		MinParament[i] = Optimal_gt->gt_array[i] - R_Range;
	}
	for (int i = 3;i < 7;i++)
	{
		MaxParament[i] = Optimal_gt->gt_array[i] + T_Range;
		MinParament[i] = Optimal_gt->gt_array[i] - T_Range;
	}
}
int sgn(double d) { return d<0 ? -1 : d>0; }   //符号函数

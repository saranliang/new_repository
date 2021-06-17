#include<iostream>
#include"Robost.h"

hash_t hash_(char const* str)
{
   hash_t ret{basis};

   while(*str){
       ret ^= *str;
       ret *= prime;
       str++;
   }

   return ret;
}
//编译时将字符串计算为哈希常量
constexpr hash_t hash_compile_time(char const* str, hash_t last_value = basis)
{
    return *str ? hash_compile_time(str+1, (*str ^ last_value) * prime) : last_value;
}
//定义运算符重载
constexpr unsigned long long operator "" _hash(char const* p, size_t)
{
    return hash_compile_time(p);
}

//平均权值
void uniform_weight(Eigen::VectorXf& r) {
    r = Eigen::VectorXf::Ones(r.rows());
}

//P阶范式权值，误差越大，权值越小；P值越大，y'=a*x^(a-1)，y'=a^x*lna
//p=1时，约等于距离的倒数
void pnorm_weight(Eigen::VectorXf& r, float p, float reg = 1e-8) {
    for (int i = 0; i<r.rows(); ++i) {
        r(i) = p / (std::pow(r(i), 2 - p) + reg);
    }

}


//按阈值部分截断，部分降低
void tukey_weight(Eigen::VectorXf& r, float p) {
    for (int i = 0; i<r.rows(); ++i) {
        if (r(i) > p) r(i) = 0.0;
        else r(i) = std::pow((1.0 - std::pow(r(i) / p, 2.0)), 2.0);
    }
}
//Huber loss  :小误差时是二次的，大误差时是线性的(对异常值不敏感)   --误差的理想均值是0.75mm
void huber_weight(Eigen::VectorXf& r, float p) {
    for (int i = 0; i<r.rows(); ++i) {
        if (r(i) > p) r(i) = p*(r(i)-p/2.0);
        else r(i) = std::pow(r(i), 2.0)/2.0;
    }
//    for (int i = 0; i<r.rows(); ++i) {
//        if (r(i) > p) r(i) = p*r(i);
//        else r(i) = std::pow(r(i), 2.0);
//    }
}
void median_wegiht(Eigen::VectorXf& r, float p)
{
    std::sort(r.data(), r.data()+r.size(), [](float lhs, float rhs){return rhs > lhs;});
    float median = r(r.size()/2);
    for(int i=0;i<r.size();i++)
    {
        if(r(i)<median) r(i) = 1.0;
        else r(i) = median/r(i);
    }

}
//反向部分截断,误差的根号
void dis_tukey_weight(Eigen::VectorXf& r, float p) {
    for (int i = 0; i<r.rows(); ++i) {
        if (r(i) < p) r(i) = 0.0;
        else r(i) = std::sqrt(r(i));
    }
}

void fair_weight(Eigen::VectorXf& r, float p) {
    for (int i = 0; i<r.rows(); ++i) {
        r(i) = 1.0 / (1.0 + r(i) / p);
    }
}

void logistic_weight(Eigen::VectorXf& r, float p) {
    for (int i = 0; i<r.rows(); ++i) {
        r(i) = (p / r(i))*std::tanh(r(i) / p);//双曲函数
    }
}


//按比例截断法，将所有点对按照配准距离排序，选取前百分之P的点对权值为1，剩余为0
//修改之后r代表的是权值，不再是距离！！！！这里有问题
void trimmed_weight(Eigen::VectorXf& r, float p) {
    std::vector<std::pair<int, float> > sortedDist(r.rows());
    for (int i = 0; i<r.rows(); ++i) {
        sortedDist[i] = std::pair<int, float>(i, r(i));
    }
    std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
    r.setZero();
    int nbV = r.rows()*p;
    for (int i = 0; i<nbV; ++i) {
        r(sortedDist[i].first) = 1.0;
    }
    std::cout << "舍弃的点为：";
    for (int i = nbV;i < r.rows();i++)
    {
        std::cout << sortedDist[i].first << " ";
    }
    std::cout << std::endl;
}


//反向P阶，误差越大，权值越大
void  dis_pnorm_weight(Eigen::VectorXf& r, float p, float reg = 1e-8){
    for (int i = 0; i<r.rows(); ++i) {
        r(i) =  (std::pow(r(i), 2 - p) + reg)/p;
    }
}
//激活函数的一种，值域为-1，1，奇函数，单调递增
void tanh_weight(Eigen::VectorXf& r, float p)
{
     for (int i = 0; i<r.rows(); ++i) {
        r(i) =  std::tanh(r(i) / p);
    }
}
//根据点簇整体误差来分配权重
//@setSize:每个元素表示一个点簇内点的个数
//点簇误差越大，权重越大；点簇内点误差越大，权重越小
void  set_weight(Eigen::VectorXf& r, float p,const std::vector<int>& setSize)
{

   Eigen::VectorXf  set_error(setSize.size());
   float one_set_sum=0.0;
   int j=0;
   int count =0;
    for(int n=0;n< r.size();n++)
    {
        one_set_sum+= r(n);
        if(count==(setSize[j]-1))  //若统计完一个点簇的误差
        {
            set_error[j] = one_set_sum/(float)(count+1);  //取每个点簇的平均值
             one_set_sum =0.0;
             j++;
             count = 0;
             continue;         //continue只跳过循环体内部的语句，但循环结构中的n++仍然执行
        }
        count++;
    }
    std::cout<<"点簇误差--------------"<<std::endl;
    std::cout<<set_error<<std::endl;
    //根据点簇误差分配给其对应的误差
    //正向权重设置
     dis_pnorm_weight(set_error,p);   //怎么决定每个点簇应该有的权重大小
     Eigen::VectorXf set_normalized = set_error/ set_error.sum();        //归一化，所有点的权值之和为1
     std::cout<<"权重--------------"<<std::endl;
    std::cout<<set_normalized<<std::endl;

    Eigen::VectorXf slice ;
    count =0;
    for(int n=0;n<set_error.size();n++)
    {
        slice =  r.segment(count,setSize[n]);    //Eigen::vector的切片操作，（start，num）,获取每个点簇对应的切片
        // 正向权重设置
        pnorm_weight(slice,p);
        //r.segment(count,setSize[n]) = set_normalized(n)*slice/ slice.sum();  //每个点簇内的权值的总和 = 之前计算的点簇的权值
        r.segment(count,setSize[n]) = Eigen::VectorXf::Ones(setSize[n])*set_normalized[n];
        count+=setSize[n];
    }
}

//注意该方法需要定义的静态常量运算符与调用在同一个文件中--------否则需要在链接时才能正确执行，而不是在编译时
 Parameters::Parameters(std::string RobostName,float p)
 {
     const char *robost_name = RobostName.c_str();    //将字符串转换为静态字符数组
     switch (hash_(robost_name)){
     case "PNORM"_hash: f = PNORM; break;
     case "TUKEY"_hash:f =TUKEY; break;
     case "FAIR"_hash: f = FAIR; break;
     case "LOGISTIC"_hash: f =LOGISTIC; break;
     case "TRIMMED"_hash: f = TRIMMED; break;
     case"HUBER"_hash:f = HUBER;break;
     case"MEDIAN"_hash:f = MEDIAN;break;
     case "NONE"_hash: f = NONE; break;
     case "DIS_TUKEY"_hash: f = DIS_TUKEY; break;
     case "DIS_PNORM"_hash:f = DIS_PNORM; break;
     case "TANH"_hash:f = TANH; break;
     case "SET"_hash:f = SET; break;
     default: f = PNORM; break;
         }
     this->p = p;
 }
 void robust_weight(Function f, Eigen::VectorXf& r, float p,const std::vector<int>& setSize) {
     //统计每个点簇的误差总和
//      std::cout<<"所有点误差-----------------"<<std::endl;
//      std::cout<<r<<std::endl;
     switch (f) {
     case PNORM: pnorm_weight(r, p); break;
     case TUKEY: tukey_weight(r, p); break;
     case DIS_TUKEY: dis_tukey_weight(r,p);break;
     case FAIR: fair_weight(r, p); break;
     case LOGISTIC: logistic_weight(r, p); break;
     case TRIMMED: trimmed_weight(r, p); break;
     case HUBER:huber_weight(r,p);break;
     case MEDIAN:median_wegiht(r,p);break;
     case NONE: uniform_weight(r); break;
     case DIS_PNORM:dis_pnorm_weight(r, p); break;
     case TANH:tanh_weight(r, p); break;
     case SET:set_weight(r,p,setSize);break;
     default: uniform_weight(r); break;
     }
     r =r/ r.sum();
//     std::cout<<"所有点权重-----------------"<<std::endl;
//     std::cout<<r<<std::endl;
 }


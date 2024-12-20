#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

    // 参数设置
    const double F = 1.20; // 变异因子F
    // 变异因子可作自调节
    const double cr = 0.6; // 交叉因子cr
    const int Size = 30;   // 种群大小
    const int CodeL = 2;   // 参数维度 (kp, kd)
    const double MinX[] = {0, 0}; // 参数最小值
    const double MaxX[] = {20, 1}; // 参数最大值
    const int maxGenerations = 50; // 最大迭代次数
    // 预测周期数G和采样时间ts
    const int G = 500;
    const double ts = 0.001; // 采样时间
    std::vector<double> BsJ_kg(maxGenerations); // 存储每一代的最佳性能指标
    std::vector<double> kp(maxGenerations); // 存储每一代的最佳kp
    std::vector<double> kd(maxGenerations); // 存储每一代的最佳kd
// 定义控制器参数结构体，用于存储比例增益(kp)和微分增益(kd)
struct ControllerParams 
{
    double kp; // 比例增益
    double kd; // 微分增益
};

// 定义PID控制器类
class DeEvolutePIDController 
{
public:
    // 构造函数：初始化控制器参数
    DeEvolutePIDController()
    {

    };
    // 静态函数：评估种群适应度函数
    double simulatePlant(const ControllerParams& kx);

    // 静态函数：执行差分进化算法以优化控制器参数
    void differentialEvolution();

private:
    // 静态私有函数：计算性能指标Ji并累积到B中
    double calculatePerformanceIndex(const std::vector<double>& e, const std::vector<double>& u);

};

#endif // PIDCONTROLLER_H
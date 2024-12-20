#include "PIDController.h"
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

// 计算性能指标
double DeEvolutePIDController::calculatePerformanceIndex(const std::vector<double>& e, const std::vector<double>& u) 
{
    double B = 0; // 初始化性能指标为0
    for (size_t i = 0; i < e.size(); ++i) 
    {
        double Ji = 0.999 * abs(e[i]) + 0.01 * pow(u[i], 2) * 0.1; // 基于最优控制理论的性能指标计算公式
        B += Ji; // 累积性能指标
        if (e[i] < 0) // 如果误差为负，则增加惩罚项
        {
            B += 10 * abs(e[i]);
        }
    }
    return B; // 返回总性能指标
}

// 评估种群适应度函数
double DeEvolutePIDController::simulatePlant(const ControllerParams& kx) 
{
    // 初始化历史状态变量
    double u_1 = 0, u_2 = 0; // 控制输入的历史值
    double y_1 = 0, y_2 = 0; // 输出的历史值
    double e_last = 0;          // 误差的历史值

    std::vector<double> yd(G, 1.0); // 目标轨迹设为阶跃信号
    std::vector<double> y(G); // 输出向量
    std::vector<double> e(G); // 误差向量
    std::vector<double> de(G); // 误差变化率向量
    std::vector<double> u(G); // 控制输入向量
    // 主循环：模拟离散时间系统的响应
    // 预测未来状态
    for (int k = 0; k < G; ++k) 
    {
        yd[k] = 1.0; // 目标位置设定为常数 1

        // 计算当前时刻的输出 y(k)
        y[k] = kx.kp * e[k] + kx.kd * de[k]; // 应用PD控制律

        // 计算误差 e(k)
        e[k] = yd[k] - y[k];

        // 计算误差变化率 de(k)
        if (k > 0)
        {
            de[k] = (e[k] - e_last) / ts; // 当不是第一次迭代时计算误差变化率
        }
        else
        {
            de[k] = 0; // 第一次迭代时没有前一时刻的误差
        }
        // PID 控制律
        u[k] = kx.kp * e[k] + kx.kd * de[k]; // 应用PD控制律

        // 更新历史状态
        u_2 = u_1; u_1 = u[k];
        y_2 = y_1; y_1 = y[k];
        e_last = e[k];
    }

    // 返回性能指标
    return calculatePerformanceIndex(e, u); // 调用性能指标计算函数并返回结果
}

// 差分进化算法主函数
void DeEvolutePIDController::differentialEvolution() 
{
    // 初始化种群
    std::vector<std::vector<double>> kxi(Size, std::vector<double>(CodeL)); // 创建种群矩阵
    // rd() 是一个 std::random_device 对象的调用，用来提供种子（seed）
    std::random_device rd; // 获取随机数生成器
    // 基于梅森旋转算法（Mersenne Twister）的伪随机数生成器对象gen
    std::mt19937 gen(rd()); // 使用Mersenne Twister生成器
    // 使用 gen 作为随机数生成器，并通过 dis 分布对象从 [0, 1] 区间内生成一个均匀分布的浮点数。
    std::uniform_real_distribution<> dis(0, 1); // 定义均匀分布
    
    // 初始化种群
    for (int i = 0; i < CodeL; ++i) 
    {
        for (auto& individual : kxi) 
        {
            individual[i] = MinX[i] + (MaxX[i] - MinX[i]) * dis(gen); // 初始化每个个体的参数
        }
    }
    // 寻找初始种群最优个体 BestS
    ControllerParams BestS = {kxi[0][0], kxi[0][1]};
    double BsJ = simulatePlant(BestS); // 对第一个个体进行评估

    for (int i = 1; i < Size; ++i) 
    {
        ControllerParams temp = {kxi[i][0], kxi[i][1]};
        double tempBsJ = simulatePlant(temp); // 对其他个体进行评估
        if (tempBsJ < BsJ)  // 如果找到更好的个体则更新最优解 
        { 
            BestS = temp;
            BsJ = tempBsJ;
        }
    }

    // 差分进化主循环
    BsJ_kg.empty(); //  存储每一代的最佳性能指标
    kp.empty(); // 存储每一代的最佳kp
    kd.empty(); // 存储每一代的最佳kd
    for (int kg = 0; kg < maxGenerations; ++kg) 
    {
        for (int i = 0; i < Size; ++i) 
        {
            // 确保 r1, r2, r3, r4 不相同且不同于当前个体索引 i
            std::vector<int> indices(Size);
            std::iota(indices.begin(), indices.end(), 0); // 填充序列0到Size-1
            std::shuffle(indices.begin(), indices.end(), gen); // 打乱序列顺序
            int r1 = indices[0], r2 = indices[1], r3 = indices[2], r4 = indices[3]; // 选择不同的随机索引

            // 变异：生成新个体 h
            ControllerParams h = {BestS.kp + F * (kxi[r1][0] - kxi[r2][0]),
                                  BestS.kd + F * (kxi[r1][1] - kxi[r2][1])}; // 根据变异规则生成新个体

            // 边界检查
            // std::clamp是 C++ 标准库中的一个函数模板，它的主要功能是将一个值限制在一个指定的范围内。
            // 这个范围由一个下限和一个上限来定义。
            h.kp = std::clamp(h.kp, MinX[0], MaxX[0]); // 确保kp在允许范围内
            h.kd = std::clamp(h.kd, MinX[1], MaxX[1]); // 确保kd在允许范围内

            // 交叉：生成试验向量 v
            ControllerParams v = {kxi[i][0], kxi[i][1]}; // 初始化v为当前个体
            for (int j = 0; j < CodeL; ++j) 
            {
                // static_cast<int> 是C++中的一种类型转换操作符，用于将一种类型安全地转换为另一种类型
                if (dis(gen) < cr) //|| j == static_cast<int>(dis(gen) * CodeL)) // 根据交叉概率决定是否交叉
                {
                    v = {h.kp, h.kd}; // 交叉后更新h->v
                }
            }

            // 选择：比较试验向量与原个体的表现
            double currentJ = simulatePlant({kxi[i][0], kxi[i][1]}); // 获取当前个体性能指标
            double trialJ = simulatePlant(v); // 获取试验向量性能指标
            if (trialJ < currentJ) 
            { // 如果试验向量表现更好
                kxi[i] = {v.kp, v.kd}; // 更新种群中当前的个体
                // 更新全局最优个体
                double tempBsJ = simulatePlant(v); // 再次评估更新后的个体
                if (tempBsJ < BsJ) 
                { // 如果是新的最优解
                    BestS = v;
                    BsJ = tempBsJ;
                }
            }
        }

        // 记录每一代的最佳 kp, kd 和性能指标
        kp[kg] = BestS.kp;
        kd[kg] = BestS.kd;
        BsJ_kg[kg] = BsJ;

        // 显示当前最佳解
        std::cout << "Generation " << kg + 1 << ": BestS = [" << BestS.kp << ", " << BestS.kd << "]" << std::endl;
    }

    // 输出最终结果
    std::cout << "Optimal PD parameters:" << std::endl;
    std::cout << "kp = " << BestS.kp << ", kd = " << BestS.kd << std::endl;
}
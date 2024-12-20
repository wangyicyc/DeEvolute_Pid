# 差分进化PID控制器优化
## 项目概述
- 本项目实现了基于差分进化（Differential Evolution, DE）算法的PID控制器参数优化。PID控制器广泛应用于工业自动化和控制系统中，用于调节温度、速度、位置等过程变量，以达到期望的设定值。传统的PID控制器参数通常是通过经验法则或试错法确定的，而这种方法可能无法在所有操作条件下都提供最优性能，特别是在非线性或时变系统中。差分进化是一种启发式的全局优化算法，非常适合用来优化像PID控制器这样复杂系统的参数。

## 目录结构
```
diff_evolution_pid/
├── README.md                   # 项目文档
├── PIDController.h             # PID控制器类头文件
├── PIDController.cpp           # PID控制器类实现文件
├── main.cpp                    # 主程序入口
├── CMakeLists.txt              # CMake构建文件（可选）
└── data/                       # 存储数据和结果的目录（可选）
```
## 安装与依赖
### 环境准备
- 编译器：需要支持C++11或更高版本的编译器，如GCC、Clang。
- 构建工具：推荐使用CMake进行项目构建。
- 构建步骤
运行程序后，差分进化算法将自动优化PID控制器的比例增益（Kp）和微分增益（Kd）。程序输出包括每一代的最佳解和最终优化后的PD参数。

## 代码结构解释
### PIDController.h
- 定义了PID控制器类及其接口，包括模拟植物模型的函数simulatePlant和执行差分进化算法的函数differentialEvolution。

### PIDController.cpp
- 实现了PIDController.h中声明的函数。具体来说：
- simulatePlant：模拟离散时间系统的响应，并返回性能指标。
- differentialEvolution：执行差分进化算法，寻找最优的PD参数。
- main.cpp：作为程序的入口点，调用PIDController::differentialEvolution()启动优化过程。

### 参数设置
在PIDController.cpp中，你可以调整以下参数来适应不同的应用场景：

F：变异因子，通常取值在[0.5, 2]之间。
cr：交叉概率，通常取值在[0.1, 0.9]之间。
Size：种群大小，决定了每次迭代中的个体数量。
CodeL：参数维度，对于PD控制器为2（Kp和Kd）。
MinX 和 MaxX：参数的最小值和最大值，限制了搜索空间。
maxGenerations：最大迭代次数，决定了算法的最大运行代数。
结果分析
程序运行结束后，会输出每一代的最佳解，并在最后显示优化后的PD参数。根据实际应用需求，可以进一步分析这些参数对控制系统性能的影响。
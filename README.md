# 目的
使机械臂以较优轨迹完成某项任务

## 1."机械臂" —— 一种运动的载体，可控的高自由度运动/力矩的模型
- [是一种manipulator：] 力学manipulator(与遥控器等发送信号的manipulator区分)
- [实现：] 目前使用6轴的ur5机械臂
## 2."较优" —— 所谓优化
- [何为较优？] 由一系列数学指标定义，数值越小被认为更优；
- [如何寻优？] 寻找，用的是搜索算法，并非暴力穷举，而是启发式搜索（这个也是核心创新点）
- [实现：] bso算法，目前几乎还没怎么改进
## 3."轨迹" —— 优化的对象
- [与路径的区别：] 路径是空间上的假想线段，和具体的物体无关；轨迹是真实存在的（不过要算上时间维度），而且
- [与动作的关系：] 某一动作可看成由一系列轨迹片段所组成；对动作的描述更抽象，而对轨迹的描述更具体，这点在规划时可以看出
- [性质：] 轨迹需连续，并至少二阶可导（现实中力的连续性）
- [建模：] {
    - [解析式类：] 建模成关于时间的初等函数
    - [神经网络：] RNN为主，实际上是去拟合一个自动机的状态转移函数。此自动机训练完后，就由其根据状态生成轨迹。
    - [层面：] 可以在位置层面建模，也可以在速度和加速度层面建模
}
- [实现：] 目前使用的是5次多项式
## 4."任务"
- []
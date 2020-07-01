## 简介
使机械臂以较优轨迹完成某项任务
###任务
- 子任务1：path-tracking。给定一条路径，使机械臂的末端沿其运动
- 子任务2：obstacle-avoidance。是机械臂在运动过程中避开障碍物
###架构
Matlab作为模型端，用于对机械臂和场景进行建模，而CoppeliaSim仿真平台(旧称v-rep)则作为仿真端，对规划算法的规划结果进行实际的仿真。
用CoppeliaSim自带的远程API客户端插件进行matlab端与CoppeliaSim平台的通信。
## 目录结构
- **/analyze** 
存放针对优化过程进行展示的脚本
- **/app**
将整个工程打包成app的产物
- **/connectionTool**
用于进行matlab端和CoppeliaSim端通信的工具
- **/data**
几个全局变量的工厂方法，进行了一些基本的初始化
- **/execute**
执行阶段的脚本
- **/experiment**
乱七八糟的测试脚本
- **/figure**
生成图像的脚本以及结果
- **/initial**
初始化阶段的脚本
- **/modular**
存放核心内容(BSO算法、适应度函数)，以及工具函数
- **/planner**
规划阶段的脚本，以及封装好的规划算法(planner_ap1.m和planner_ap2.m)
- **/vrep-lua**
存放CoppeliaSim中用到的一些lua脚本
- **./**
当前目录下，main.m是一个简单的顶层执行脚本，start_up.m用来导入一些必需的路径，UR5.ttt是CoppeliaSim端的场景文件
##规划流程

##注意
###命名
- _p2p后缀是点对点规划(point-to-point)的代称，这个任务只是一开始当脚手架用的，现已遗弃。
- _ap后缀是Along-Path的缩写(其实现在正式的名称是path-tracking...)，这个是正式的任务
###org
.org文件上记着的一些待办事项比较混乱，且已经过时了
###第三方依赖
openGJK 和 matlab robotic toolbox

# EE5112 Project 2 – STOMP Collision Avoidance

## 环境准备

- MATLAB R2023a 或更新版本，启用 Robotics System Toolbox、Robotics System Toolbox Support Package；
- 将仓库加入 MATLAB path，确保 `EE5112_Project_2_Collision-Avoidance-Code` 为当前工作目录；
- 运行前先执行 `helperCreateObstaclesKINOVA` 内部脚本生成体素地图（Live Script 会自动调用）。

## 核心脚本

- `RunLiveScript.m`：一键执行 Live Script。
- `KINOVA_STOMP_Path_Planning.mlx`：Live Script，展示加载机器人、IK 求解、STOMP 规划和可视化。
- 主要函数位于 `EE5112_Project_2_Collision-Avoidance-Code`：
  - `helperSTOMP.m`：完整的 STOMP 迭代流程；
  - `stompSamples.m`、`stompDTheta.m`、`stompObstacleCost.m`、`stompTrajCost.m`：采样与代价计算；
  - `helperComputePoEData.m`、`updateJointsWorldPosition.m`、`vecToSe3.m`：PoE 正运动学实现；
  - `helperCreateObstaclesKINOVA.m`：更复杂的障碍场；
  - 其他辅助文件 `stompRobotSphere.m`、`stompUpdateProb.m`、`stompUpdateTheta.m` 等。

## 测试步骤

1. 在 MATLAB 中将工作目录切换到 `EE5112_Project_2_Collision-Avoidance-Code`。
2. 运行 `RunLiveScript` 或直接打开并执行 `KINOVA_STOMP_Path_Planning.mlx`。
3. 观察命令行输出的 `Qtheta`、`RAR` 收敛情况，确认迭代未提前因错误退出。
4. 查看生成的动画窗口：
   - 初始场景、障碍布局、目标姿态；
   - STOMP 训练过程（可将 `enableVideoTraining` 设为 1 并查看 `Iiwa14_Training.avi`）；
   - 终端轨迹播放（可设置 `enableVideo = 1` 导出 `Iiwa14_wEEConY3.avi`）。
5. 脚本结尾会保存规划好的关节轨迹为 `Theta_nDisc20_nPaths_20.mat`，可加载检查终态碰撞状态 (`isTrajectoryInCollision` 应为 false)。

## 如果出现问题

- IK 失败：调整 `taskInit`、`taskFinal` 姿态或修改权重；
- STOMP 不收敛：调节 `nPaths`、`convergenceThreshold` 或 `orientationConstraintCost` 权重；
- 障碍布局过密导致采样失败：修改 `helperCreateObstaclesKINOVA` 中的障碍尺寸 / 位置。

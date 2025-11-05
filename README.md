# EE5112 Project 2 – STOMP Collision Avoidance

## 通用准备
- 安装 MATLAB R2023a 及以上版本，勾选 Robotics System Toolbox 和其 Support Package。
- 在 MATLAB 中将项目根目录加入 path，并把 `EE5112_Project_2_Collision-Avoidance-Code` 设为当前工作目录。
- 建议先运行 `helperCreateObstaclesKINOVA`（或整套 Live Script），确认体素地图和碰撞体定义均能正常生成。

---

## Task 1 · 跑通官方 STOMP 示例
1. 打开 `KINOVA_STOMP_Path_Planning.mlx` 并逐段运行，或直接执行 `RunLiveScript.m`。
2. 关注命令行输出的 `Qtheta`、`RAR`，确保随着迭代收敛；若报错集中检查：
   - `helperSTOMP.m`（主循环）
   - `stompSamples.m`、`stompDTheta.m`、`stompObstacleCost.m`、`stompTrajCost.m`
3. 运行完成后确认：
   - 动画中 Kinova 机械臂成功绕开原始立方体障碍；
   - `isTrajectoryInCollision` 为 `false`；
   - 输出文件 `Theta_nDisc20_nPaths_20.mat` 包含最终轨迹。

---

## Task 2 · 改用 KUKA LBR iiwa14
1. 在 Live Script 中确认 `robot_name = 'kukaIiwa14'`，末端通过 `robot.BodyNames{end}` 指定。
2. 检查 `taskInit`、`taskFinal` 是否在 iiwa14 的可达范围内，可使用 `show(robot, currentRobotJConfig)` 预览。
3. 再次运行 Live Script，确认更换机械臂后依旧能得到无碰撞轨迹。

---

## Task 3 · 使用 PoE 正运动学
1. `helperComputePoEData.m` 会预先求出螺旋轴和零姿态矩阵，在 `helperSTOMP.m` 中自动调用。
2. `updateJointsWorldPosition.m` 改为调用 PoE：
   ```matlab
   poeData = helperComputePoEData(robot, endEffector);
   [X, ~] = updateJointsWorldPosition(robot, currentRobotJConfig, poeData);
   ```
   对比 `getTransform` 或 `show` 的结果，误差应在数值精度内。
3. 在报告中写明螺旋轴的来源（几何雅可比、URDF 参数、PoE 理论等）。

---

## Task 4 · 自定义更复杂的避障场景
1. 修改 `helperCreateObstaclesKINOVA.m` 中 `obstacleSpecs` 的中心和尺寸；当前默认已包含三块障碍体，可按需要扩展。
2. 执行 Live Script，确认初始轨迹确实会与障碍冲突，STOMP 规划后能够绕开。
3. 若需展示过程，可在 `helperSTOMP.m` 中将 `enableVideoTraining` 或 `enableVideo` 设为 1，生成 `Iiwa14_Training.avi`、`Iiwa14_wEEConY3.avi`。

---

## Task 5 · 末端姿态约束（保持 y 轴竖直）
1. `stompTrajCost.m` 内的 `orientationConstraintCost` 对末端 y 轴与世界 z 轴的偏差加 L1 罚，默认权重为 100，可按需求调整。
2. 对比无约束（将罚项权重设为 0）与有约束两种运行结果，记录轨迹与动画差异。
3. 在报告/展示中说明约束实现方式及其对路径规划的影响。

---

## 最终验证清单
- [ ] `KINOVA_STOMP_Path_Planning.mlx` 全流程无报错，`isTrajectoryInCollision = false`。
- [ ] PoE 输出与 MATLAB 自带前向运动学结果一致。
- [ ] 自定义障碍正确渲染，轨迹绕开并满足安全距离。
- [ ] 姿态约束前后轨迹差异已记录（图像或动画）。
- [ ] 报告对应章节说明每个 Task 的实现与测试。

若遇问题，可按需调整：
- IK 失败：修改 `taskInit` / `taskFinal` 或 IK 权重；
- STOMP 停滞：调节 `nPaths`、`convergenceThreshold`、平滑权重或约束系数；
- 障碍过密：在 `helperCreateObstaclesKINOVA.m` 中放宽体素边界或减少障碍数量。

完成以上步骤即可逐项验证并交付整套 Project 2 要求。祝顺利！

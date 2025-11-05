# EE5112 Project 2 – STOMP Collision Avoidance

## 通用准备
- 安装 MATLAB R2023a 及以上版本，勾选 Robotics System Toolbox 和其 Support Package。
- 在 MATLAB 中将项目根目录加入 path，并把 `EE5112_Project_2_Collision-Avoidance-Code` 设为当前工作目录。
- 建议先运行 `helperCreateObstaclesKINOVA`（或整套 Live Script），确认体素地图和碰撞体定义均能正常生成。

---

## Task 1 · 跑通官方 STOMP 示例
目标：按照课程提供的示例代码，完成缺失函数并让 Kinova Gen3 成功规划无碰撞轨迹。

必做步骤  
1. 打开 `KINOVA_STOMP_Path_Planning.mlx`，逐节执行（或在命令行运行 `RunLiveScript`）。  
2. 在执行过程中重点检查以下函数是否已补全并能正常运行：  
   - `helperSTOMP.m`：采样、概率、梯度和更新流程；  
   - `stompSamples.m`：对每个关节独立高斯采样；  
   - `stompDTheta.m`：根据轨迹概率计算梯度估计；  
   - `stompObstacleCost.m`：基于 sEDT 的障碍代价；  
   - `stompTrajCost.m`：整合局部/全局代价。  
3. 关注命令行的 `Qtheta`、`RAR` 输出应随迭代下降并收敛。  
4. 运行结束后确认：  
   - 动画窗口展示 Kinova 绕开默认立方体；  
   - `isTrajectoryInCollision` = `false`；  
   - 生成 `Theta_nDisc20_nPaths_20.mat` 并含有最终关节轨迹。  

交付物与记录  
- 截图或录屏展示成功运行的 Live Script 与动画；  
- 在报告中说明上述关键函数的实现思路或修改点；  
- 若有调参（如 `nPaths`、`convergenceThreshold`），请记录参数。

---

## Task 2 · 改用 KUKA LBR iiwa14
目标：在 Task 1 基础上换用另一款机械臂，并保证 STOMP 仍然成功。

必做步骤  
1. 在 Live Script 中设置：`robot_name = 'kukaIiwa14'`，末端名称为 `robot.BodyNames{end}`。  
2. 更新 `taskInit`、`taskFinal`，确保姿态在 iiwa14 的可达空间内。必要时在单独脚本调用 `show(robot, q)` 检查初始姿态。  
3. 重新执行 Task 1 的全部流程，验证 iiwa14 同样能完成避障。  

交付物与记录  
- 运行日志或动画截图，显示换臂后规划结果；  
- 记录 IK 初末姿态（位置和欧拉角），在报告中说明如何选择这些姿态；  
- 如需额外调节关节限制或权重，请说明原因。

---

## Task 3 · 使用 PoE 正运动学
目标：替换 MATLAB 内置 FK 计算，实现基于 Product of Exponentials 的前向运动学。

必做步骤  
1. 运行 `helperComputePoEData` 生成 `poeData`：其中应包含 `Slist`（螺旋轴）、`Mlist`（零位变换）。  
2. `updateJointsWorldPosition.m` 使用下列方式验证：  
   ```matlab
   poeData = helperComputePoEData(robot, endEffector);
   [Xpoe, Tpoe] = updateJointsWorldPosition(robot, currentRobotJConfig, poeData);
   Tref = cell(1,numel(robot.BodyNames));
   for i = 1:numel(robot.BodyNames)
       Tref{i} = getTransform(robot, currentRobotJConfig, robot.BodyNames{i});
   end
   % 对比 Tpoe 与 Tref
   ```  
3. 若差异超过 1e-6，请检查 `vecToSe3.m`、`helperComputePoEData.m`、螺旋轴顺序等实现。  
4. 在报告中写清：  
   - 螺旋轴的提取方式（如利用 `geometricJacobian`）；  
   - PoE 实现与验证方法；  
   - 对效率或数值稳定性的考虑。

交付物与记录  
- 比对表格或数值误差截图；  
- 代码片段或注释说明 PoE 推导步骤；  
- 若借鉴外部库，请注明来源。

---

## Task 4 · 自定义更复杂的避障场景
目标：在默认环境基础上构建更具挑战性的碰撞场景。

必做步骤  
1. 编辑 `helperCreateObstaclesKINOVA.m` 的 `obstacleSpecs`，自定义多个障碍（碰撞体类型可扩展为 `collisionCylinder`、`collisionSphere` 等）。  
2. 确保初始插值轨迹（未优化的 `theta`）会与障碍产生冲突，否则需调整障碍位置或初末姿态。  
3. 执行 Live Script，验证 STOMP 能在新场景下找到无碰撞路径。  
4. 如需直观展示改动，可开启：  
   ```matlab
   enableVideoTraining = 1;
   enableVideo = 1;
   ```  
   并检查生成的 `Iiwa14_Training.avi` 与 `Iiwa14_wEEConY3.avi`。

交付物与记录  
- 环境截图（初态、关键中间帧、终态）；  
- 新障碍配置表（位置、尺寸、类型）；  
- 若添加特殊几何体或导入外部模型，请记录来源与转换方式。

---

## Task 5 · 末端姿态约束（保持 y 轴竖直）
目标：在规划过程中保证末端执行器特定方向保持竖直（如端杯场景）。

必做步骤  
1. 检查 `stompTrajCost.m` 中 `orientationConstraintCost`，确认默认对末端 y 轴与世界 z 轴的差异施加 L1 罚。  
2. 若需其他约束（如保持 x 轴水平），可在函数内改用所需方向向量或追加多个罚项。  
3. 为对比效果，可分别运行：  
   - 保持默认罚权重（例：100）；  
   - 将罚权重设为 0（无约束）。  
   记录两种情况下的姿态及路径差异。  
4. 检查在受限情况下，末端姿态是否满足所需精度（例如误差角度 < 5 度）。  

交付物与记录  
- 两组轨迹/动画对比；  
- 误差量化（末端方向向量与期望方向的夹角或 L1/L2 值）；  
- 报告中说明约束如何实现及对轨迹的影响。

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

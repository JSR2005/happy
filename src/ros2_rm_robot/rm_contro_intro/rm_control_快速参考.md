# rm_control 模块 - 快速参考指南

## 一句话总结
**rm_control** 是轨迹平滑引擎：接收MoveIt的稀疏轨迹 → 用三次样条插值扩展成密集轨迹 → 以50Hz频率平稳发送给硬件。

---

## 核心文件位置

| 文件 | 位置 | 功能 |
|------|------|------|
| rm_control.h | `include/rm_control/rm_control.h` | 类定义、成员变量、回调函数声明 |
| rm_control.cpp | `src/rm_control.cpp` | 核心实现（插值、发送逻辑） |
| cubicSpline.h | `include/rm_control/cubicSpline.h` | 三次样条插值算法 |

---

## 关键类和函数

### Rm_Control 类 (ROS2节点)

```cpp
class Rm_Control : public rclcpp::Node {
    // 构造函数：初始化Action服务器、定时器、发布器
    Rm_Control();
    
    // 核心函数1：轨迹插值线程
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    
    // 核心函数2：50Hz定时回调，周期性发送
    void timer_callback();
    
    // 其他回调函数
    void get_move_stop_callback(const std_msgs::msg::Empty::SharedPtr msg);
};
```

### cubicSpline 类 (数值算法)

```cpp
class cubicSpline {
    // 加载数据并计算样条系数
    bool loadData(double *x_data, double *y_data, int count, 
                  double bound1, double bound2, BoundType type);
    
    // 根据时间点查询位置、速度、加速度
    bool getYbyX(double &x_in, double &y_out);
    
private:
    // 核心算法：追赶法求解三对角方程组
    bool spline(BoundType type);
};
```

---

## 核心工作流

### Phase 1: 轨迹接收 (MoveIt → execute_move)

```
MoveIt规划完成
     ↓
发送Action: FollowJointTrajectory
     ↓
rm_control::execute_move() 被触发
```

### Phase 2: 轨迹插值 (execute_move)

```cpp
// 步骤1：从MoveIt消息提取数据
for (i = 0; i < point_num; i++) {
    p_joint1[i] = goal->trajectory.points[i].positions[0];
    v_joint1[i] = goal->trajectory.points[i].velocities[0];
    a_joint1[i] = goal->trajectory.points[i].accelerations[0];
    time_from_start[i] = goal->trajectory.points[i].time_from_start.sec + 
                         goal->trajectory.points[i].time_from_start.nanosec/1e9;
}

// 步骤2：对每个关节进行三次样条插值
cubicSpline spline;
double max_time = time_from_start[point_num-1];

// Joint1 插值
if (spline.loadData(time_from_start, p_joint1, point_num, 0, 0, 
                    cubicSpline::BoundType_First_Derivative)) {
    x_out = -rate;
    while(x_out < max_time) {
        x_out += rate;  // 每次增加20ms
        spline.getYbyX(x_out, y_out);
        time_from_start_.push_back(x_out);
        p_joint1_.push_back(y_out);
        v_joint1_.push_back(vel);
        a_joint1_.push_back(acc);
    }
}
// Joint2~7 类似...

// 步骤3：标记数据已准备
p2.vector_len = time_from_start_.size();  // 如240
p2.vector_cnt = 0;
point_changed = true;

// 步骤4：等待定时器发送完成
while(point_changed) {
    // 睡眠2ms，避免忙轮询
}
```

### Phase 3: 轨迹发送 (timer_callback 50Hz)

```cpp
void Rm_Control::timer_callback() {
    if(point_changed) {
        if(p2.vector_cnt < p2.vector_len) {
            // 发送轨迹点
            if(arm_type_ == 75) {  // 7轴
                joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt);
                // ... joint2~7
            } else {  // 6轴
                joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt);
                // ... joint2~6
            }
            this->joint_pos_publisher->publish(joint_msg);
            p2.vector_cnt++;  // 指向下一个点
        }
        else {
            // 所有点发送完，进入保持阶段
            if(count_final_joint <= count_keep_send) {
                // 继续发送最后一个点 (1.5秒)
                joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt-1);
                // ...
                this->joint_pos_publisher->publish(joint_msg);
                count_final_joint++;
            }
            else {
                // 完成，重置状态
                count_final_joint = 0;
                point_changed = false;  // 通知execute_move完成
            }
        }
    }
}
```

### Phase 4: 硬件执行 (rm_driver → 机械臂)

```
rm_control 发布
     ↓
/rm_driver/movej_canfd_cmd (ROS2话题)
     ↓
rm_driver 接收
     ↓
TCP 192.168.1.188:8081 (发送命令)
     ↓
机械臂硬件控制器
     ↓
关节运动执行
     ↓
UDP 192.168.1.188:9501 (实时反馈 50Hz)
```

---

## 全局变量说明

| 变量 | 类型 | 初值 | 说明 |
|------|------|------|------|
| `time_from_start_[]` | vector<double> | [] | 插值后的时间戳数组 |
| `p_joint1_[]~p_joint7_[]` | vector<double> | [] | 各关节位置 |
| `v_joint1_[]~v_joint7_[]` | vector<double> | [] | 各关节速度 |
| `a_joint1_[]~a_joint7_[]` | vector<double> | [] | 各关节加速度 |
| `p2.vector_len` | int | 0 | 轨迹总点数 |
| `p2.vector_cnt` | int | 0 | 当前发送点索引 |
| `point_changed` | bool | false | 轨迹状态标志 |
| `rate` | double | 0.020 | 定时器周期 (50Hz) |
| `count_final_joint` | int | 0 | 保持发送计数 |

---

## 三次样条插值核心概念

### 问题
- MoveIt 规划了 10-20 个路点
- 直接发送这么少的点会导致轨迹折线状，有尖刺

### 解决方案
- 使用三次样条插值将 10-20 个点扩展到 1000+ 个
- 保证位置、速度、加速度都连续光滑

### 算法
1. **loadData()** - 接收原始数据，使用追赶法求解三对角方程组，计算每个点的二阶导数
2. **getYbyX()** - 查询任意时间点的位置、速度、加速度

### 效果
```
插值前：●————●————●————●————●    (5个点，折线）
插值后：●····●····●····●····●   (240个点，光滑曲线)
```

---

## 性能指标

| 指标 | 数值 |
|------|------|
| 轨迹发送频率 | 50 Hz |
| 时间步长 | 20 ms |
| 单条轨迹点数 | 1000+ |
| 单条轨迹耗时 | 2-8 秒 |
| 完成后保持时间 | 1.5 秒 |
| 网络延迟 | <1 ms |

---

## 常见问题

### Q1: 为什么需要保持发送1.5秒？
**A**: 确保机械臂有足够时间到达目标位置，避免轨迹执行完但机械臂还在运动时接收新轨迹导致冲突。

### Q2: rate=0.020为什么设置为20ms？
**A**: 对应50Hz采样频率，是硬件控制器的实时反馈频率。同步采样避免通信延迟。

### Q3: point_changed标志的作用？
**A**: 线程间同步标志。execute_move 根据此标志判断何时所有数据都已发送完成，然后返回成功结果。

### Q4: 如何支持其他轴数？
**A**: 添加更多 p_jointN_, v_jointN_, a_jointN_ 向量，在 loadData 循环中增加关节数，在 joint_msg 打包中处理新关节。

---

## 调试建议

### 问题1：轨迹执行不动
```
检查清单：
1. ros2 node list 是否有 rm_driver 节点
2. ros2 topic list 是否有 /rm_driver/movej_canfd_cmd 话题
3. 机械臂硬件是否上电
4. 网络连接是否正常
```

### 问题2：轨迹有抖动
```
解决方案：
1. 减小 rate 值（如改为 0.010 = 100Hz）
2. 检查网络延迟（ping 192.168.1.188）
3. 检查MoveIt速度规划是否过快
```

### 问题3：偶发执行失败
```
原因分析：
1. MoveIt 规划的路点数 ≤ 3
2. 轨迹太复杂导致插值失败
解决方案：
1. 增加 MoveIt 的路点数
2. 简化轨迹规划
```

---

## 代码位置速查

| 查找内容 | 位置 |
|---------|------|
| 三次样条插值算法 | rm_control.cpp: 第100~300行 (loadData/spline/getYbyX) |
| 轨迹接收处理 | rm_control.cpp: 第345行 (execute_move) |
| 轨迹插值循环 | rm_control.cpp: 第465~600行 (6个嵌套if-else) |
| 周期发送逻辑 | rm_control.cpp: 第790行 (timer_callback) |
| 保持发送阶段 | rm_control.cpp: 第860~900行 |
| Action服务器定义 | rm_control.h: 第50~70行 |
| 全局数据定义 | rm_control.cpp: 第1~50行 |

---

## 最佳实践

1. **始终验证轨迹点数**
   ```cpp
   if (point_num > 3) {  // 有效轨迹
       // 进行插值
   }
   ```

2. **使用适当的插值精度**
   ```cpp
   // 标准：20ms (50Hz)
   double rate = 0.020;
   
   // 如果需要更光滑：改为10ms (100Hz)
   double rate = 0.010;
   ```

3. **监控状态变量**
   ```cpp
   RCLCPP_INFO(this->get_logger(), 
               "Trajectory points: %d, interpolated to %d points",
               point_num, p2.vector_len);
   ```

4. **错误处理**
   ```cpp
   if (!rclcpp::ok() || goal_handle->is_canceling()) {
       result->error_code = -1;
       result->error_string = "Goal was canceled";
       goal_handle->canceled(result);
       return;
   }
   ```

---

## 相关文件导航

- **本文档的详细版本**：`rm_control_详细分析.md`
- **时序图和流程**：`rm_control_时序图.md`
- **系统架构全景**：`系统架构分析.md`
- **Force-Position控制示例**：`api_Force_Position_Control_demo.cpp`

---

## 下一步学习

1. 阅读 `rm_driver.h` 理解硬件驱动层
2. 研究 `cubicSpline.h` 深入三次样条算法
3. 调试运行一个简单的轨迹执行任务
4. 修改插值参数观察效果变化

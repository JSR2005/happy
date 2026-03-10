# Unity 与 ROS2 集成完整指南

## 第一部分：环境配置

### 1.1 System Requirements

```
操作系统: Windows 10/11 或 Linux (Ubuntu 22.04)
Unity 版本: 2021 LTS 或更新 (推荐 2022 LTS)
.NET 版本: .NET 6.0 或更新
Python: 3.8+ (用于 ROS2 发送数据)
内存: 16GB+ (开发)
```

### 1.2 安装 ROS2 for Unity

**步骤1: 在 Unity Hub 中新建项目**

```
Unity Hub → New Project → 3D (URP)
项目名: RobotControlUI
存储路径: 任意
```

**步骤2: 添加 NuGet 支持**

Unity 本身不自带 NuGet，需要通过以下方式集成：

```bash
# 方法A: 使用 OpenUPM (推荐)
# 在项目目录下执行
openupm add io.github.roboticsToolkit.ros2_for_unity

# 方法B: 手动下载并导入
# 从 GitHub 下载: https://github.com/RoboticsToolkit/ros2-for-unity
# 解压到 Assets/ROS2/ 文件夹
```

**步骤3: 配置 .NET Framework**

在 Unity 编辑器中：
```
Edit → Project Settings → Player
→ Configuration
→ API Compatibility Level: .NET Framework 4.x
```

### 1.3 安装必要的 NuGet 包

编辑 `Assets/ROS2/packages.config` 或通过 NuGet Package Manager:

```xml
<?xml version="1.0" encoding="utf-8"?>
<packages>
  <package id="rcldotnet" version="0.2.0" />
  <package id="rcldotnet_common" version="0.2.0" />
  <package id="Newtonsoft.Json" version="13.0.3" />
  <package id="WebSocketSharp" version="1.0.2-rc11" />
</packages>
```

---

## 第二部分：ROS2 WebSocket Server 实现

### 2.1 创建 ROS2 WebSocket 桥接节点 (Python)

```python
# unity_bridge/unity_bridge_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import asyncio
import json
import websockets
from datetime import datetime

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from vision_interfaces.msg import DetectionArray

class UnityBridgeNode(Node):
    def __init__(self):
        super().__init__('unity_bridge')
        
        # 参数
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('publish_frequency', 50)  # Hz
        
        ws_port = self.get_parameter('websocket_port').value
        freq = self.get_parameter('publish_frequency').value
        
        # WebSocket 服务器
        self.ws_server = None
        self.connected_clients = set()
        
        # 缓存最新数据
        self.current_joint_states = None
        self.current_force = None
        self.current_pose = None
        self.current_detections = None
        
        # QoS 设置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建订阅器
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            sensor_qos
        )
        
        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/force_feedback',
            self.force_callback,
            sensor_qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            sensor_qos
        )
        
        self.detection_sub = self.create_subscription(
            DetectionArray,
            '/detection_result',
            self.detection_callback,
            sensor_qos
        )
        
        # 定时发送数据给 Unity
        period = 1.0 / freq  # Convert Hz to seconds
        self.timer = self.create_timer(
            period, 
            self.broadcast_to_unity
        )
        
        # 启动 WebSocket 服务器
        self.ws_port = ws_port
        self.get_logger().info(f'启动 WebSocket 服务器在 ws://0.0.0.0:{ws_port}')
        
        # 在异步任务中运行 WebSocket 服务器
        self.ws_task = asyncio.create_task(self.run_websocket_server())
    
    def joint_state_callback(self, msg):
        self.current_joint_states = {
            'position': list(msg.position),
            'velocity': list(msg.velocity),
            'effort': list(msg.effort),
            'timestamp': self.get_clock().now().to_msg().sec
        }
    
    def force_callback(self, msg):
        self.current_force = {
            'force': {
                'x': msg.wrench.force.x,
                'y': msg.wrench.force.y,
                'z': msg.wrench.force.z
            },
            'torque': {
                'x': msg.wrench.torque.x,
                'y': msg.wrench.torque.y,
                'z': msg.wrench.torque.z
            },
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
    
    def pose_callback(self, msg):
        self.current_pose = {
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            },
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
    
    def detection_callback(self, msg):
        detections = []
        for det in msg.detections:
            detections.append({
                'class_id': det.class_id,
                'confidence': det.confidence,
                'bbox': {
                    'x': det.bbox_x,
                    'y': det.bbox_y,
                    'width': det.bbox_width,
                    'height': det.bbox_height
                }
            })
        
        self.current_detections = {
            'detections': detections,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
    
    def broadcast_to_unity(self):
        """构造并广播消息给所有 Unity 客户端"""
        message = {
            'timestamp': datetime.now().isoformat(),
            'robot_state': self.current_joint_states,
            'force_feedback': self.current_force,
            'tcp_pose': self.current_pose,
            'detections': self.current_detections
        }
        
        # 异步发送给所有连接的客户端
        if self.connected_clients:
            asyncio.create_task(self._send_to_all_clients(json.dumps(message)))
    
    async def _send_to_all_clients(self, message):
        """发送消息给所有连接的客户端"""
        disconnected = set()
        
        for client in self.connected_clients:
            try:
                await client.send(message)
            except Exception as e:
                self.get_logger().error(f'发送失败: {e}')
                disconnected.add(client)
        
        self.connected_clients -= disconnected
    
    async def run_websocket_server(self):
        """运行 WebSocket 服务器"""
        async with websockets.serve(self.websocket_handler, '0.0.0.0', self.ws_port):
            await asyncio.Future()  # 永久运行
    
    async def websocket_handler(self, websocket, path):
        """处理单个客户端连接"""
        self.connected_clients.add(websocket)
        self.get_logger().info(f'客户端连接: {websocket.remote_address}')
        
        try:
            async for message in websocket:
                # 接收来自 Unity 的命令
                try:
                    cmd = json.loads(message)
                    self.get_logger().debug(f'收到命令: {cmd}')
                    
                    # 处理不同的命令类型
                    if cmd.get('type') == 'gripper_command':
                        # 转发夹爪命令给相应的发布器
                        pass
                    elif cmd.get('type') == 'motion_command':
                        # 转发运动命令
                        pass
                    
                except json.JSONDecodeError:
                    self.get_logger().error(f'无法解析 JSON: {message}')
        
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connected_clients.discard(websocket)
            self.get_logger().info(f'客户端断开: {websocket.remote_address}')

def main(args=None):
    rclpy.init(args=args)
    node = UnityBridgeNode()
    
    # 混合同步和异步运行
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.2 启动脚本

```bash
#!/bin/bash
# 启动 WebSocket 桥接节点

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run unity_bridge unity_bridge_node \
    --ros-args \
    -p websocket_port:=9090 \
    -p publish_frequency:=50
```

---

## 第三部分：Unity C# 脚本

### 3.1 ROS2 连接管理器

```csharp
// Assets/Scripts/ROS2ConnectionManager.cs
using UnityEngine;
using WebSocketSharp;
using Newtonsoft.Json.Linq;

public class ROS2ConnectionManager : MonoBehaviour {
    [SerializeField] private string serverURL = "ws://localhost:9090";
    [SerializeField] private bool autoConnect = true;
    
    private WebSocket ws;
    private bool isConnected = false;
    
    // 单例模式
    private static ROS2ConnectionManager instance;
    
    public delegate void ConnectionStatusChanged(bool connected);
    public static event ConnectionStatusChanged OnConnectionStatusChanged;
    
    public delegate void DataReceived(JObject data);
    public static event DataReceived OnDataReceived;
    
    void Start() {
        if (instance == null) {
            instance = this;
            DontDestroyOnLoad(gameObject);
        } else {
            Destroy(gameObject);
            return;
        }
        
        if (autoConnect) {
            Connect();
        }
    }
    
    public void Connect() {
        if (isConnected) return;
        
        try {
            ws = new WebSocket(serverURL);
            
            ws.OnOpen += () => {
                Debug.Log("[ROS2] 已连接到服务器");
                isConnected = true;
                OnConnectionStatusChanged?.Invoke(true);
            };
            
            ws.OnMessage += (sender, e) => {
                try {
                    JObject message = JObject.Parse(e.Data);
                    OnDataReceived?.Invoke(message);
                } catch (System.Exception ex) {
                    Debug.LogError($"[ROS2] 解析消息失败: {ex.Message}");
                }
            };
            
            ws.OnError += (sender, e) => {
                Debug.LogError($"[ROS2] WebSocket 错误: {e.Message}");
            };
            
            ws.OnClose += (sender, e) => {
                Debug.LogWarning("[ROS2] 连接已关闭");
                isConnected = false;
                OnConnectionStatusChanged?.Invoke(false);
            };
            
            ws.Connect();
        } catch (System.Exception ex) {
            Debug.LogError($"[ROS2] 连接失败: {ex.Message}");
        }
    }
    
    public void Disconnect() {
        if (ws != null && isConnected) {
            ws.Close();
        }
    }
    
    public void SendCommand(string command, JObject parameters) {
        if (!isConnected) {
            Debug.LogWarning("[ROS2] 未连接，无法发送命令");
            return;
        }
        
        JObject msg = new JObject();
        msg["type"] = command;
        msg["parameters"] = parameters;
        
        ws.Send(msg.ToString());
    }
    
    public bool IsConnected => isConnected;
    
    void OnDestroy() {
        if (ws != null) {
            ws.Close();
        }
    }
}
```

### 3.2 机械臂模型控制脚本

```csharp
// Assets/Scripts/RobotController.cs
using UnityEngine;
using Newtonsoft.Json.Linq;
using System.Collections.Generic;

[System.Serializable]
public class JointTransform {
    public Transform transform;
    public Vector3 rotationAxis;  // 旋转轴
}

public class RobotController : MonoBehaviour {
    [SerializeField] private JointTransform[] joints = new JointTransform[7];
    [SerializeField] private Transform tcpTransform;
    [SerializeField] private float jointAngleSmoothFactor = 0.1f;
    
    private float[] targetJointAngles = new float[7];
    private float[] currentJointAngles = new float[7];
    
    // DH 参数（用于正向运动学）
    private readonly Vector3[] DH_a = new Vector3[7];  // 连杆长度
    private readonly float[] DH_d = new float[7];      // 连杆偏移
    private readonly float[] DH_alpha = new float[7];  // 扭转角
    
    void Start() {
        // 初始化 DH 参数（RM 75 机械臂的参数）
        InitializeDHParameters();
        
        // 订阅 ROS2 数据
        ROS2ConnectionManager.OnDataReceived += OnROS2DataReceived;
    }
    
    void OnDestroy() {
        ROS2ConnectionManager.OnDataReceived -= OnROS2DataReceived;
    }
    
    void Update() {
        // 平滑更新关节角度
        for (int i = 0; i < 7; ++i) {
            currentJointAngles[i] = Mathf.Lerp(
                currentJointAngles[i],
                targetJointAngles[i],
                jointAngleSmoothFactor * Time.deltaTime * 50  // 假设 50Hz 更新
            );
        }
        
        // 更新机械臂视觉模型
        UpdateRobotVisualization();
    }
    
    void OnROS2DataReceived(JObject data) {
        try {
            if (data["robot_state"] != null && data["robot_state"]["position"] != null) {
                var positions = data["robot_state"]["position"].ToObject<float[]>();
                
                for (int i = 0; i < positions.Length && i < 7; ++i) {
                    // 转换弧度到角度
                    targetJointAngles[i] = positions[i] * Mathf.Rad2Deg;
                }
            }
        } catch (System.Exception ex) {
            Debug.LogError($"[Robot] 更新关节状态失败: {ex.Message}");
        }
    }
    
    void UpdateRobotVisualization() {
        // 分层次更新每个关节
        // link0 (base) -> link1 -> link2 -> ... -> link7 -> TCP
        
        Matrix4x4 currentTransform = Matrix4x4.identity;
        
        for (int i = 0; i < 7; ++i) {
            if (joints[i].transform == null) continue;
            
            // 围绕旋转轴旋转
            Quaternion rotation = Quaternion.AngleAxis(
                currentJointAngles[i],
                joints[i].rotationAxis
            );
            
            joints[i].transform.localRotation = rotation;
            
            // 累积变换
            currentTransform = currentTransform * 
                Matrix4x4.TRS(joints[i].transform.localPosition, 
                             rotation, 
                             Vector3.one);
        }
        
        // 更新 TCP 位置（末端执行器）
        if (tcpTransform != null) {
            tcpTransform.position = 
                joints[6].transform.position + 
                joints[6].transform.rotation * new Vector3(0, 0, 0.15f);  // 0.15m 末端距离
        }
    }
    
    void InitializeDHParameters() {
        // RM 75 的 DH 参数（示意值，需要根据实际机械臂调整）
        DH_d[0] = 0.329f;
        DH_d[1] = 0.0f;
        DH_d[2] = 0.0f;
        DH_d[3] = 0.375f;
        DH_d[4] = 0.0f;
        DH_d[5] = 0.0f;
        DH_d[6] = 0.15f;
    }
    
    public void UpdateRobotPose(float[] jointAngles) {
        for (int i = 0; i < jointAngles.Length && i < 7; ++i) {
            targetJointAngles[i] = jointAngles[i];
        }
    }
}
```

### 3.3 力反馈图表

```csharp
// Assets/Scripts/ForceChartUI.cs
using UnityEngine;
using UnityEngine.UI;
using Newtonsoft.Json.Linq;
using System.Collections.Generic;

public class ForceChartUI : MonoBehaviour {
    [SerializeField] private Image forceChartImage;
    [SerializeField] private Texture2D chartTexture;
    [SerializeField] private int historySize = 200;
    
    private Queue<Vector3> forceHistory = new Queue<Vector3>();
    private Queue<Vector3> torqueHistory = new Queue<Vector3>();
    private Texture2D runtimeChartTexture;
    
    private readonly Color[] axisColors = {
        Color.red,    // X - 红
        Color.green,  // Y - 绿
        Color.blue    // Z - 蓝
    };
    
    void Start() {
        // 创建运行时图表纹理
        int width = 400;
        int height = 200;
        runtimeChartTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        
        // 清空纹理
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                runtimeChartTexture.SetPixel(x, y, Color.black);
            }
        }
        runtimeChartTexture.Apply();
        
        forceChartImage.texture = runtimeChartTexture;
        
        // 订阅数据
        ROS2ConnectionManager.OnDataReceived += OnForceDataReceived;
    }
    
    void OnForceDataReceived(JObject data) {
        try {
            if (data["force_feedback"] != null && 
                data["force_feedback"]["force"] != null) {
                
                var forceData = data["force_feedback"]["force"];
                Vector3 force = new Vector3(
                    (float)forceData["x"],
                    (float)forceData["y"],
                    (float)forceData["z"]
                );
                
                // 添加到历史记录
                forceHistory.Enqueue(force);
                if (forceHistory.Count > historySize) {
                    forceHistory.Dequeue();
                }
                
                // 重绘图表
                RedrawChart();
            }
        } catch (System.Exception ex) {
            Debug.LogError($"[ForceChart] 更新失败: {ex.Message}");
        }
    }
    
    void RedrawChart() {
        // 清空纹理
        int width = runtimeChartTexture.width;
        int height = runtimeChartTexture.height;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                runtimeChartTexture.SetPixel(x, y, Color.black);
            }
        }
        
        // 绘制坐标轴
        for (int x = 0; x < width; ++x) {
            runtimeChartTexture.SetPixel(x, height / 2, new Color(0.2f, 0.2f, 0.2f));
        }
        
        // 绘制力曲线
        int pixelIndex = 0;
        foreach (var force in forceHistory) {
            if (pixelIndex >= width) break;
            
            // 正规化力值到像素范围 (-50N to +50N)
            int y_x = (int)(height / 2 + (force.x / 50.0f) * (height / 2));
            int y_y = (int)(height / 2 + (force.y / 50.0f) * (height / 2));
            int y_z = (int)(height / 2 + (force.z / 50.0f) * (height / 2));
            
            y_x = Mathf.Clamp(y_x, 0, height - 1);
            y_y = Mathf.Clamp(y_y, 0, height - 1);
            y_z = Mathf.Clamp(y_z, 0, height - 1);
            
            // 绘制三轴
            runtimeChartTexture.SetPixel(pixelIndex, y_x, axisColors[0]);  // X - 红
            runtimeChartTexture.SetPixel(pixelIndex, y_y, axisColors[1]);  // Y - 绿
            runtimeChartTexture.SetPixel(pixelIndex, y_z, axisColors[2]);  // Z - 蓝
            
            pixelIndex++;
        }
        
        runtimeChartTexture.Apply();
    }
    
    void OnDestroy() {
        ROS2ConnectionManager.OnDataReceived -= OnForceDataReceived;
        Destroy(runtimeChartTexture);
    }
}
```

### 3.4 控制面板 UI

```csharp
// Assets/Scripts/ControlPanelUI.cs
using UnityEngine;
using UnityEngine.UI;
using Newtonsoft.Json.Linq;

public class ControlPanelUI : MonoBehaviour {
    [SerializeField] private Button gripperOpenButton;
    [SerializeField] private Button gripperCloseButton;
    [SerializeField] private Slider gripperWidthSlider;
    
    [SerializeField] private Dropdown controlModeDropdown;
    [SerializeField] private Toggle forceControlToggle;
    
    [SerializeField] private Text statusText;
    [SerializeField] private Text connectionStatusText;
    
    void Start() {
        // 夹爪按钮
        gripperOpenButton.onClick.AddListener(OnGripperOpen);
        gripperCloseButton.onClick.AddListener(OnGripperClose);
        gripperWidthSlider.onValueChanged.AddListener(OnGripperWidthChanged);
        
        // 控制模式
        controlModeDropdown.onValueChanged.AddListener(OnControlModeChanged);
        forceControlToggle.onValueChanged.AddListener(OnForceControlToggled);
        
        // 订阅连接状态
        ROS2ConnectionManager.OnConnectionStatusChanged += OnConnectionStatusChanged;
        ROS2ConnectionManager.OnDataReceived += OnDataReceived;
    }
    
    void OnGripperOpen() {
        var cmd = new JObject();
        cmd["gripper_width"] = 0.0f;  // 完全打开
        
        ROS2ConnectionManager.instance.SendCommand("gripper_command", cmd);
        statusText.text = "夹爪打开...";
    }
    
    void OnGripperClose() {
        var cmd = new JObject();
        cmd["gripper_width"] = 1.0f;  // 完全关闭
        
        ROS2ConnectionManager.instance.SendCommand("gripper_command", cmd);
        statusText.text = "夹爪关闭...";
    }
    
    void OnGripperWidthChanged(float value) {
        var cmd = new JObject();
        cmd["gripper_width"] = value;
        
        ROS2ConnectionManager.instance.SendCommand("gripper_command", cmd);
    }
    
    void OnControlModeChanged(int index) {
        string[] modes = { "手动", "自动抓取", "力控制", "学习模式" };
        statusText.text = $"控制模式: {modes[index]}";
    }
    
    void OnForceControlToggled(bool value) {
        if (value) {
            statusText.text = "启用力控制";
        } else {
            statusText.text = "禁用力控制";
        }
    }
    
    void OnConnectionStatusChanged(bool connected) {
        if (connected) {
            connectionStatusText.text = "✓ 已连接";
            connectionStatusText.color = Color.green;
        } else {
            connectionStatusText.text = "✗ 未连接";
            connectionStatusText.color = Color.red;
        }
    }
    
    void OnDataReceived(JObject data) {
        // 可以在这里更新 UI 显示的状态信息
        // 例如关节角度、力反馈等
    }
    
    void OnDestroy() {
        ROS2ConnectionManager.OnConnectionStatusChanged -= OnConnectionStatusChanged;
        ROS2ConnectionManager.OnDataReceived -= OnDataReceived;
    }
}
```

---

## 第四部分：构建 Unity 场景

### 4.1 场景层次结构

在 Unity Hierarchy 中创建以下结构：

```
Scene/
├─ RobotModel/
│  ├─ Base (空对象)
│  ├─ Link1-7 (Cube/Cylinder 网格)
│  └─ EndEffector (夹爪模型)
├─ Cameras/
│  ├─ MainCamera (Scene View)
│  └─ RobotCamera (俯视图)
├─ Canvas/
│  ├─ ControlPanel
│  ├─ ForceChart
│  ├─ StatusDisplay
│  └─ ConnectionStatus
└─ Managers/
   ├─ ROS2ConnectionManager (脚本)
   ├─ RobotController (脚本)
   └─ UIManager (脚本)
```

### 4.2 关键配置

```
ROS2ConnectionManager 设置:
- Server URL: ws://localhost:9090
- Auto Connect: true

RobotController 设置:
- Joint Count: 7
- TCP Transform: 指定 EndEffector
- Joint Smooth Factor: 0.1

ForceChartUI 设置:
- History Size: 200
- Chart Size: 400x200
- Axis Colors: RGB
```

---

## 第五部分：故障排查

### 问题1: Unity 无法连接到 ROS2

**症状**: `ConnectionStatusText` 显示 "未连接"

**解决方案**:
```bash
# 1. 检查 ROS2 WebSocket 服务是否运行
netstat -tuln | grep 9090

# 2. 检查防火墙
sudo ufw allow 9090

# 3. 检查 ROS2 节点
ros2 node list

# 4. 测试连接
wscat -c ws://localhost:9090
```

### 问题2: 机械臂模型不动

**症状**: 场景中的 Robot 模型没有运动

**解决方案**:
```csharp
// 在 RobotController 中添加调试输出
void OnROS2DataReceived(JObject data) {
    Debug.Log($"[RobotController] 收到数据: {data}");
    
    if (data["robot_state"] != null) {
        Debug.Log($"[RobotController] 关节状态: {data["robot_state"]}");
    }
}
```

### 问题3: WebSocket 连接频繁断开

**症状**: `ConnectionStatus` 闪烁

**解决方案**:
- 增加 `publish_frequency` 以避免超时
- 检查网络延迟 (`ping localhost`)
- 增加 WebSocket 的心跳间隔

---

## 第六部分：性能优化

### 优化建议

```csharp
// 1. 限制更新频率
void Update() {
    // 只在特定帧数更新
    if (Time.frameCount % 2 == 0) {  // 每隔一帧更新
        UpdateRobotVisualization();
    }
}

// 2. 使用对象池存储历史数据
private Queue<Vector3> forceHistory;

// 3. 减少 GC 压力
JObject data;
if (data.TryGetValue("robot_state", out var state)) {
    // 使用 TryGetValue 而不是 try-catch
}

// 4. 使用线程池处理耗时计算
ThreadPool.QueueUserWorkItem(ComputeFK, jointAngles);
```

---

## 总结

这个集成方案实现了：
✓ ROS2 与 Unity 的实时通信
✓ 机械臂 3D 模型同步显示
✓ 力/扭矩实时图表
✓ 远程控制接口

**预计开发时间**: 1-2周（如果有基础的 Unity 和 C# 经验）

**后续扩展方向**:
- 添加相机视频流显示
- 实现手臂运动录制和回放
- 添加VR/AR交互
- 数据记录和分析

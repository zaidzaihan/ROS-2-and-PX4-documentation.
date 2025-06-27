# Beginner's Guide to ROS 2 for PX4 Development

A comprehensive step-by-step guide to get started with PX4 drone development using ROS 2. This guide is designed for beginners and provides practical examples to build your understanding progressively.

## 📋 Table of Contents

- [Prerequisites](#prerequisites)
- [Installation Guide](#installation-guide)
- [Learning Path](#learning-path)
- [Hands-on Tasks](#hands-on-tasks)
- [Troubleshooting](#troubleshooting)
- [Additional Resources](#additional-resources)

## 🎯 Prerequisites

Before starting this guide, ensure you have:

1. **Ubuntu 22.04 LTS (Jammy Jellyfish)** - Recommended OS for PX4 development
2. **Basic Linux command line knowledge** - You should be comfortable with terminal operations
3. **Programming knowledge** - Familiarity with either Python or C++
4. **At least 100GB free disk space** - PX4 installation can exceed 40GB

## 🛠️ Installation Guide

### Step 1: Ubuntu 22.04 LTS Setup

1. Download the [Ubuntu 22.04 Desktop image](https://releases.ubuntu.com/jammy/)
2. Create a [dual boot system](https://www.freecodecamp.org/news/how-to-dual-boot-windows-10-and-ubuntu-linux-dual-booting-tutorial/) (recommended)
3. Allocate **at least 100GB** for the Ubuntu partition

### Step 2: Install ROS 2 Humble

1. Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. **Test your installation** by running the [Talker-Listener example](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#talker-listener)

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2  
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

### Step 3: Install QGroundControl

Download and install [QGroundControl for Ubuntu](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu) for UAV management and monitoring.

### Step 4: PX4 and Micro XRCE-DDS Setup

1. Read the [PX4 ROS 2 Introduction](https://docs.px4.io/main/en/ros2/)
2. Follow the [Official PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html)
   - **Skip** the "Install ROS 2" section (already completed)
   - Focus on setting up the **Micro XRCE-DDS Agent and Client**

## 📚 Learning Path

Complete these tutorials in order to build your foundation:

### Phase 1: ROS 2 CLI Tools

Complete the [Beginner: CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) tutorials:

- Start with [Configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- Continue through [Launching nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)

### Phase 2: ROS 2 Client Libraries

Complete the [Beginner: Client Libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html) tutorials:

- Start with [Using `colcon` to build packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- Continue through [Using parameters in a class](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)

> **Note:** Choose either **Python** or **C++** based on your preference and stick with it throughout the tutorials.

### Phase 3: PX4 Integration

1. Read through the [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html)
2. Understand how to write ROS nodes that communicate with PX4
3. Complete the [ROS 2 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control.html)

> **Important:** Always run QGroundControl when testing PX4 simulations, and remember to run `colcon build` after making code changes.

## 🎮 Hands-on Tasks

These practical tasks will deepen your understanding of ROS 2 and PX4 integration:

### Task 1: Turtle-Controlled Drone 🐢➡️🚁

**Objective:** Control a PX4 drone simulation using turtlesim's turtle_teleop_key controls.

**What you'll learn:**
- Topic subscription and publishing
- Message type conversion
- Real-time drone control

**Key Topics Used:**
- `turtle1/pose` - Contains position (x, y, theta) and velocity data
- `turtle1/cmd_vel` - Contains velocity commands

[View detailed implementation →](#task-1-implementation)

### Task 2: Custom uORB Messages 📨

**Objective:** Create custom uORB topics to send data from ROS 2 to PX4.

**What you'll learn:**
- uORB message creation
- ROS 2 to PX4 communication
- Message bridging with µXRCE-DDS

**Required Reading:**
- [PX4 Middleware Documentation](https://docs.px4.io/main/en/middleware/uorb.html)
- [µXRCE-DDS Bridge Documentation](https://docs.px4.io/main/en/middleware/uxrce_dds.html#supported-uorb-messages)

[View detailed implementation →](#task-2-implementation)

### Task 3: MAVLink Integration 📡

**Objective:** Develop MAVLink messages for custom uORB topics and verify transmission with Wireshark.

**What you'll learn:**
- MAVLink protocol
- Message streaming
- Network analysis with Wireshark
- WLua plugin development

**Tools Required:**
- Wireshark
- Custom WLua plugin

[View detailed implementation →](#task-3-implementation)

## 🔧 Task Implementations

### Task 1 Implementation

<details>
<summary>Click to expand detailed steps</summary>

#### Setup Workspace

```bash
# Navigate to your workspace
cd ~/ws_offboard_control/src/

# Create new package
ros2 pkg create --build-type ament_python --license Apache-2.0 control_py
```

#### Key Code Components

1. **Import Required Messages:**
```python
from turtlesim.msg import Pose
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
```

2. **Subscribe to Turtle Pose:**
```python
self.turtle_pose_subscriber = self.create_subscription(
    Pose, '/turtle1/pose', self.turtle_pose_callback, qos_profile)
```

3. **Control Logic:**
```python
def timer_callback(self):
    if self.vehicle_local_position.z > self.takeoff_height:
        self.publish_position_setpoint(
            self.turtle_pose.x, 
            self.turtle_pose.y, 
            self.takeoff_height, 
            self.turtle_pose.theta
        )
```

#### Running the System

You'll need 5 terminals running simultaneously:
1. PX4 Simulation: `make px4_sitl gz_x500`
2. MicroXRCE Agent: `MicroXRCEAgent udp4 -p 8888`
3. QGroundControl: `./QGroundControl.AppImage`
4. Turtlesim: `ros2 run turtlesim turtlesim_node`
5. Teleop: `ros2 run turtlesim turtle_teleop_key`

Finally, run your controller:
```bash
cd ~/ws_offboard_control
source install/setup.bash
ros2 run control_py controller
```

</details>

### Task 2 Implementation

<details>
<summary>Click to expand detailed steps</summary>

#### Create Custom uORB Message

1. **Define Message Structure** (`PX4-Autopilot/msg/Test.msg`):
```
# Turtle pose message
uint64 timestamp
float32 x
float32 y  
float32 theta
float32 linear_velocity
float32 angular_velocity
```

2. **Update CMakeLists.txt** - Add `Test.msg` to the message list

3. **Configure DDS Bridge** (`src/modules/uxrce_dds_client/dds_topics.yaml`):
```yaml
subscriptions:
  - topic: /fmu/in/test
    type: Test
```

#### ROS 2 Publisher Node

```python
class PosePublisher(Node):
    def __init__(self):
        super().__init__("uorb_publisher")
        
        # Subscribe to turtle pose
        self.subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.pose_callback, 10)
        
        # Publish to PX4
        self.publisher = self.create_publisher(Test, '/fmu/in/test', 10)
        
        # Timer for regular publishing
        self.create_timer(2.0, self.timer_callback)
    
    def timer_callback(self):
        msg = Test()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher.publish(msg)
```

#### Verification

Check if your message is received in PX4:
```bash
# In PX4 terminal
uorb status        # List all uORB messages
listener test      # Listen to your custom message
```

</details>

### Task 3 Implementation

<details>
<summary>Click to expand detailed steps</summary>

#### Create MAVLink Message

1. **Define MAVLink Message** (`development.xml`):
```xml
<message id="45501" name="POSE_VALUES">
    <description>Mavlink test message to handle custom uorb msg</description>
    <field type="uint64_t" name="timestamp">Timestamp</field>
    <field type="float" name="x">x value</field>
    <field type="float" name="y">y value</field>
    <field type="float" name="theta">theta value</field>
    <field type="float" name="linear_velocity">linear velocity</field>
    <field type="float" name="angular_velocity">angular velocity</field>
</message>
```

2. **Create Streaming Class** (`POSE_VALUES.hpp`):
```cpp
class MavlinkStreamPoseValues : public MavlinkStream {
    // ... class implementation
    bool send() override {
        test_s pose_values{};
        if (_pose_values_sub.update(&pose_values)) {
            mavlink_pose_values_t pose_msg{};
            pose_msg.timestamp = pose_values.timestamp;
            pose_msg.x = pose_values.x;
            // ... assign other fields
            
            mavlink_msg_pose_values_send_struct(_mavlink->get_channel(), &pose_msg);
            return true;
        }
        return false;
    }
};
```

#### Wireshark Analysis

1. **Generate WLua Plugin:**
```bash
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_development message_definitions/v1.0/development.xml
```

2. **Install Plugin** in Wireshark's personal Lua plugin folder

3. **Capture and Filter:**
   - Interface: `loopback`
   - Filter: `not icmp && mavlink_proto.POSE_VALUES_timestamp`

</details>

## 🐛 Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| **Build failures** | Ensure all dependencies are installed and sourced properly |
| **Topic not found** | Check if all nodes are running and topics are published |
| **PX4 won't arm** | Verify QGroundControl is running and safety checks pass |
| **No MAVLink data** | Confirm MicroXRCE agent is running on correct port (8888) |

### Useful Commands

```bash
# Check ROS 2 topics
ros2 topic list -t
ros2 topic echo /topic_name

# Check PX4 uORB messages  
uorb status
listener topic_name

# Build specific package
colcon build --packages-select package_name

# Source workspace
source install/setup.bash
```

## 📖 Additional Resources

### Official Documentation
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [PX4 User Guide](https://docs.px4.io/main/en/)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/master/en/)

### Community Resources
- [PX4 Discourse](https://discuss.px4.io/)
- [ROS 2 Community](https://discourse.ros.org/)
- [PX4 GitHub Repository](https://github.com/PX4/PX4-Autopilot)

### Video Tutorials
- [ROS 2 Basics Playlist](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q)
- [PX4 Development Course](https://www.youtube.com/playlist?list=PLy601_yIdvSOjJCiXbj4C8lStIan7F-cC)

---

## 🤝 Contributing

Found an issue or want to improve this guide? Please:

1. Fork this repository
2. Create a feature branch
3. Submit a pull request with your improvements

## 📄 License

This guide is released under the [Apache 2.0 License](LICENSE).

---

**Happy coding! 🚁✨**

> Remember: The key to mastering ROS 2 and PX4 development is consistent practice. Start with the basics, complete each task thoroughly, and don't hesitate to experiment with the code!

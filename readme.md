# Beginner guide to ROS 2 for PX4 development  

## This guide outlines the steps to begin your PX4 development journey with ROS 2

### Prerequisites

1. **Install Ubuntu 22.04 LTS (Jammy Jellyfish)**
   - Ensure your system is set up with the necessary operating system.

2. **Install ROS 2 Humble**
   - Follow the official installation guide for ROS 2 Humble.

3. **Complete ROS 2 Humble Beginner Tutorials**
   - **CLI Tools**: Familiarize yourself with basic command-line tools.
   - **Client Libraries**: Learn the basics of writing and running ROS 2 nodes using client libraries.

4. **Install QGroundControl**
   - Set up QGroundControl for UAV management and monitoring.

5. **Follow the PX4: ROS 2 User Guide**
   - Integrate ROS 2 with PX4 by following the official user guide.

6. **Set up Micro XRCE-DDS Agent and Client**
   - Install and configure the Micro XRCE-DDS communication bridge.

7. **Build and Run a ROS 2 Workspace**
   - Set up your ROS 2 workspace and ensure it is functional by building and running a sample project.

---  

### Installing Ubuntu 22.04 LTS (Jammy Jellyfish)

- Download the **Ubuntu 22.04 Desktop image** [here](https://releases.ubuntu.com/jammy/).
- It is recommended to create a [dual boot system](https://www.freecodecamp.org/news/how-to-dual-boot-windows-10-and-ubuntu-linux-dual-booting-tutorial/) and allocate **at least 100GB** for Ubuntu 22.04 OS, as PX4 can require more than 40GB after installation.

---  

### Installing ROS 2 Humble

- Follow the ROS 2 Humble installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

**Note:** Test the installation by running the [Talker-Listener example](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#talker-listener) provided on the same page.

---  

### List of ROS 2 Beginner Tutorials

Complete both ROS 2 beginner tutorials below before proceeding, as the concepts are crucial for the next steps.

#### **1. ROS 2 Beginner: CLI Tools Guide**

- Follow the **Beginner: CLI Tools** tutorials [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html).
- Start from the [**Configuring environment**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) section and continue until [**Launching nodes**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html).

  ![CLI Tools Tutorial](img/image-1.png)

#### **2. ROS 2 Beginner: Client Libraries Guide**

- Follow the **Beginner: Client Libraries** documentation [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html).
- Start from [**Using `colcon` to build packages**](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and continue until [**Using parameters in a class**](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html).

  ![Client Libraries Tutorial](img/image.png)

**Note:** Some of the tutorials above support the use of either **Python** or **C++** for implementation. **Choose** the programming language you are **most familiar** with.

---  

### Installing and Understanding PX4 and QGC

1. Start by reading the **PX4 ROS 2** [**Introduction**](https://docs.px4.io/main/en/ros2/).

2. Follow the Official **PX4 ROS 2 guide** [**here**](https://docs.px4.io/main/en/ros2/user_guide.html) to install PX4 and set up the **Micro XRCE-DDS Agent and Client**.

    ![alt text](img/ros2_px4.png)
3. Install **QGroundControl** for Ubuntu [**here**](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu). You may read the QGC documentation on the same page.

4. Read through the guide until [**here**](https://docs.px4.io/main/en/ros2/user_guide.html#ros-2-example-applications) to understand how to **write a ROS node** to listen to topics published by PX4.

  **Note:** When following the [**guide**](https://docs.px4.io/main/en/ros2/user_guide.html), **skip** the [**Install ROS 2**](https://docs.px4.io/main/en/ros2/user_guide.html#install-ros-2) section as we have already installed it before.

- **ROS 2 Offboard Control Example**

  - Run the **ROS 2 Offboard Control Example** provided on [**this page**](https://docs.px4.io/main/en/ros2/offboard_control.html) and try experimenting with the code. Remember to run `colcon build` each time you make changes to the code to rebuild it.  
  Also dont forget to **run QGC** when trying to run the PX4 simulation.

---  

### Tasks Completion

Here are some task you should try in order to further grasp the concept of ROS 2 PX4 development.

1. **Extending turtlesim's `turtle_teleop_key` controls** to **control the drone simulation** in the ROS 2 Offboard Control Example.

2. **Creating a custom uORB topics** and using it to **send custom values** from ROS 2 to PX4.

3. **Develop Mavlink message for the custom uORB topics** and use it to **send the uORB message**. Analyze it by using WireShark to prove that the custom Mavlink is being sent.  

**It is recommended for you to try finishing the above tasks by yourself first.**  

Use the guide below only if you are stuck.

---  

### Task 1: Extending turtlesim's turtle_teleop_key node's command to control drone simulation

#### **Instruction: Use topic from turtlesim node's topic to control the PX4 drone simulation.**

  There are two turtlesim's topic that you can use to try controlling the drone which are **turtle1/pose** and **turtle1/cmd_vel**.  

  This tutorial uses turtlesim's pose topic as it can give full control in 2D vector (x, y, theta).

  To start tinkering, we first need to see the type and the values of the messages being sent from **turtlesim_node**.  
  Open a new terminal, source the ROS 2 setup file, and run **turtlesim_node**.

  `source /opt/ros/humble/setup.bash`  
  `ros2 run turtlesim turtlesim_node`

  ![alt text](img/turtlesim.png)

  Then open a new terminal, again **source the ROS 2** setup file and run:

  `ros2 topic list`

  ![topic_list](img/topic_list.png)

  We can see, there is **/turtle1/pose** topic in the list.  
  Next, echo the values from the topic.

  `ros2 topic echo /turtle1/pose`

  We can see the variables of the topic emitted are as below:  

  `x: 5.306337833404541`  
  `y: 5.2459282875061035`  
  `theta: -1.2784440517425537`  
  `linear_velocity: 0.0`  
  `angular_velocity: 0.0`  

  End the echo by using **Ctrl+C**, and try to check the type of the topic:

  `ros2 topic list -t`

  We can see the message type of the `/turtle1/pose` topic is [**Pose**](http://docs.ros.org/en/noetic/api/turtlesim/html/msg/Pose.html):  
  _/turtle1/pose [turtlesim/msg/Pose]_

  With this information, we can **subscribe to the `/turtle1/pose` topic**, and use the values to publish and move the drone to the corresponding positions.

- Assuming you have the `~/ws_offboard_control` colcon workspace directory you created while trying the [**ROS 2 Offboard Control Example**](https://docs.px4.io/main/en/ros2/offboard_control.html) before, **open a new terminal.**  

  Note: This guide uses **Python** for the development.  
  _Visit ROS 2 humble website if you want to use C++ instead._

1. In the new terminal, source the ROS 2 setup and enter the offboard_control's src folder :  

    - `source /opt/ros/humble/setup.bash`  

    - `cd ~/ws_offboard_control/src/`

2. Create a new package and name it as **control_py**.  

    - `ros2 pkg create --build-type ament_python --license Apache-2.0 control_py`

3. Open the workspace using your preferred code editor.  

    - `code .`

4. In the **control_py** folder, setup the email, name and license in **package.xml** and **setup.py** according to the previous tutorial. (See notes below)

    **Note** : Follow this [**tutorial**](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) to create a ROS 2 package if you have any problem.

5. Enter into the `control_py` folder and create a new `controller.py` python file.

    - control_py
        - controller.py

6. Copy this [**offboard_control.py**](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py) code from the **ROS 2 Offboard Control example** and paste it into **controller.py**.

7. In the code editor, open the **setup.py** file, and add the following line within the console_scripts brackets of the entry_points field:  

    ```python
    ...
    entry_points={
        'console_scripts': [
            'controller = control_py.controller: main',
        ],
    },
    ...

8. Open a terminal and head to the root of the workspace and build the package to ensure that the setup is successful.  
`cd ~/ws_offboard_control`  
`colcon build --packages-select control_py`

9. Head over to the code editor, and open **controller.py**.

10. At the top of the code, import Pose message from turtlesim message:  
  `from turtlesim.msg import Pose`

11. Update the QoS profile so that it is compatible with turtlesim pose.  

      ```python
     qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

12. Subscribe to the turtlesim's pose, and create a callback function called **turtle_pose_callback** at the subscription sections:

      ```python
      ...
        self.turtle_pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.turtle_pose_callback, qos_profile)
      ...

13. Initialize turtle_pose variables with the Pose class:

      ```python
      ...
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.turtle_pose = Pose()#initialize turtle_pose structure.
        self.takeoff_height = -5.0
      ...

14. Create the turtle_pose_callback function definition:

      ```python
          def turtle_pose_callback(self, turtle_pose):
            self.turtle_pose = turtle_pose #Callback function for turtle1/pose topic subscriber.
  
15. Lastly, we can modify the timer_callback code so that instead of making the drone take off and land after reaching the set height, we can configure it to hover at a fixed height. We would then control the drone's x and y positions and yaw angle using the values obtained from the turtlesim pose. We also needs to comment the land() function from being called so that the drone will always hover.

      ```python
          def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(
                self.turtle_pose.x, self.turtle_pose.y, self.takeoff_height, self.turtle_pose.theta)

        #elif self.vehicle_local_position.z <= self.takeoff_height:
            #self.land()
            #exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

16. Save the code, and build the package using `colcon build`. Make sure you are in the root folder of the workspace, outside the src folder.

    `colcon build --packages-select control_py`

    ![colcon_build](img/colcon_build.png)

17. If the build succeeds, open a new terminal and navigate to the **ws_offboard_control** workspace.

    `cd ~/ws_offboard_control`

18. Then source the local setup.

    `source install/setup.bash`

19. Before running the control_py controller package, we need to run PX4 simulation, MicroXRCE-DDS Agent, turtle_teleop_key, QGC and turtlesim_node.  
    1. To run PX4 simulation, open a new terminal, enter into the PX4-Autopilot folder

        `cd ~/PX4-Autopilot`

          and run:

        `make px4_sitl gz_x500`

    2. To run MicroXRCE, open a new terminal and head over to Micro-XRCE-DDS-Agent folder

        `cd Micro-XRCE-DDS-Agent/`  

        and run:
  
        `MicroXRCEAgent udp4 -p 8888`

    3. To run QGC, **head over to the folder** in which the application image is in, and run:

        `./QGroundControl.AppImage`

20. Open two new terminal and **source the ROS 2 workspace** in each terminal,  

      `source /opt/ros/humble/setup.bash`

       and in the first terminal run:  

      `ros2 run turtlesim turtlesim_node`

      and for the second terminal run:

      `ros2 run turtlesim turtle_teleop_key`

    - For reference, here is all the required tools and ROS 2 nodes needed to be run:

    ![sim1](img/sim1.png)

21. Now head over to the first terminal that is in the ws_offboard_control workspace, and run the executables from the package we created earlier.

      `ros2 run control_py controller`

22. The drone in the PX4's Gazebo simulation should now start arming and takeoff until our set height. Now, head over to the turtle_teleop_key terminal and try to control the drone using the arrow key. You can also control the yaw of the drone by using G|B|V|C|D|E|R|T keys.

![image-2](img/image-2.png)

_The image shows the drone simulation view from above. The drone's position moves according to the `turtle_teleop_key` controller, using values from the `turtle1/pose` topic in the turtlesim simulation. The drone's movement mirrors the turtle's movement in the turtlesim simulation._

---  

### Task 2: Creating a custom uORB topics and using it to send custom values from ROS 2 to PX4

Instruction: Create custom uORB messages that can **receive message from turtle1/pose topic** and **displays it** into PX4 shell/terminal or QGC Mavlink console.

Briefing:  

- Start by reading through PX4 official **middleware documentation**: <https://docs.px4.io/main/en/middleware/uorb.html>  

- Focus on the [Adding a new topic](https://docs.px4.io/main/en/middleware/uorb.html#adding-a-new-topic) section.

- As we use ROS 2 node to send data to PX4, we need to use µXRCE-DDS to bridge between the two. Head over to <https://docs.px4.io/main/en/middleware/uxrce_dds.html#supported-uorb-messages> for more info.

Step by step:

1. First, we need to see the interface of the `turtle1/pose` topic so that we can struct the uORB msg file accordingly. Start by opening a new terminal, source the ROS 2 environment and **run the turtlesim node**.  
      `source /opt/ros/humble/setup.bash`  
      `ros2 run turtlesim turtle_teleop_key`  

      ![teleop_key](img/turtle_node.png)

2. Next open a new terminal, source the workspace and list all running nodes and their types:  
      `ros2 topic list -t`  

      ![list_type](img/list_type.png)

3. Now use the command **_ros2 interface show <msg_type>_** to view the topic's interface:  
      `ros2 interface show turtlesim/msg/Pose`

      ![interface](img/interface.png)

4. We can now see the structure of the 'Pose' topic that we can use to put into our custom uORB msg. To start creating the msg, open PX4-Autopilot/msg folder.
      ![msg_folder](img/px4_msg.png)

5. Inside the msg folder, create a new file and name it into `Test.msg`.

6. Next, open CMakeLists.txt file, and put the Test.msg file name into the list where all the other msgs is in.
      ![testcmake](img/Testcmake.png)

7. Now, open the Test.msg file, and structure the message according to the turtlesim's pose interface.
      ![pose_test](img/pose_test.png)

8. Copy the Test.msg file and head over to your ROS 2 workspace and go to src folder.  
(Any workspace is permissible as long as you have/imported px4_msgs package into the workspace).

9. Head over to px4_msgs package, open it's msg folder and paste the Test.msg file into it.
      ![px4_msgs_test](img/px4_msgs_test.png)

10. Open the workspace root folder in the terminal, in this example ros2_ws, and colcon build the px4_msgs package:
      `colcon build --packages-select px4_msgs`
      ![colcon_px4_msgs](img/colcon_px4_msgs.png)

11. Now head back to **PX4-Autopilot folder**. And go to:
      `src/modules/uxrce_dds_client`
      ![px4_microfolder](img/px4_microfolder.png)

12. Open the `dds_topics.yaml` file. Since we are going to publish the Pose values msg to the PX4, PX4 needs to subscribe to the uORB message. Under the "subscriptions: " section, put the custom uORB msg that we created early using the following format for the topic and type:
      ![dds_topic_test](img/dds_topic_test.png)

13. Save the file. Now we need to create a ROS 2 node to handle the publishing of `/turtle1/pose` and publish it to the `/fmu/in/test` topic. Open the previous ROS 2 PX4 workspace and create a new package called "posepub":  
      `ros2 pkg create --build-type ament_python --license Apache-2.0 posepub`
  
14. Set up the `package.xml` and `setup.py` as usual, and create a python module inside the posepub file. For this example, the module is named as posepub.py.
      - posepub  
          - posepub.py

15. Open the posepub.py module using your preferred IDE, and begin importing the necessaries library/module and our custom PX4 uORB message:

      ```python
          import rclpy
          from rclpy.clock import Clock
          from rclpy.node import Node
          from px4_msgs.msg import Test
          from turtlesim.msg import Pose

16. Next, we create a class, `PosePublisher`, that inherits from `Node`. In this class, we set up a subscriber to the `turtlesim`'s `pose` topic, which updates local variables with the turtle's position and orientation. These variables are then assigned to our custom `Test` message. We also establish a publisher to send this message to the `/fmu/in/test` topic, periodically publishing the updated pose data.

      ```python
              class PosePublisher(Node):
                  def __init__(self):
                      super().__init__("uorb_publisher")

                      self.subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
                      self.publisher = self.create_publisher(Test, '/fmu/in/test', 10)
                      self.x = 0.0
                      self.y = 0.0
                      self.theta = 0.0
                      self.linear_velocity = 0.0
                      self.angular_velocity = 0.0
                      timer_period = 2.0
                      self.create_timer(timer_period, self.timer_callback)

                  def timer_callback(self):
                      msg = Test()
                      msg.x = self.x
                      msg.y = self.y 
                      msg.theta = self.theta
                      msg.linear_velocity = self.linear_velocity
                      msg.angular_velocity = self.angular_velocity
                      msg.timestamp = int(Clock().now().nanoseconds / 1000)
                      self.publisher.publish(msg)
                      

                  def pose_callback(self, msg):
                      self.x = msg.x
                      self.y = msg.y
                      self.theta = msg.theta
                      self.linear_velocity = msg.linear_velocity
                      self.angular_velocity = msg.angular_velocity
                      self.get_logger().info(f"Message sent!")

              def main(args=None):
                  rclpy.init(args=args)
                  uorbPub = PosePublisher()
                  rclpy.spin(uorbPub)
                  uorbPub.destroy_node()
                  rclpy.shutdown()

              if __name__ == "__main__":
                  try:
                      main()
                  except Exception as e:
                      print(e)
    ```

17. Save the file. Now open the `setup.py` and create an entry point for our node's module, in this case:

      ```python
                  entry_points={
        'console_scripts': [
            'posepub = posepub.posepub:main'
        ],
    },
    ```

18. Save the file and open the root of this workspace in a terminal. Colcon build the package:  
    `colcon build --packages-select posepub`  
    ![posepub_build](img/posepub_build.png)

19. If there is no error, proceed to a new terminal and build the PX4 using make:  
    `cd ~/PX4-Autopilot`  
    `make px4_sitl gz_x500`

20. Open two new termnial and run MicroXRCE Agent and also QGroundControl.

21. Open a new terminal and run ROS 2 turtlesim's turtlesim_node:  
    `ros2 run turtlesim turtlesim_node`

22. Finally, navigate to the root folder of our workspace, source the local setup and then run the node.  
    `source install/local_setup.bash`  
    `ros2 run posepub posepub`

    ![all_posepub](img/all_posepub.png)

23. Now head over to the terminal that we use to run the PX4 simulation, and in the terminal, use `uorb status` command to view the list of all uorb messages being received by the PX4. Check the uORB list to see if our test message is being received.

    ![test_uorb](img/test_uorb.png)

24. If our test msg is being displayed in the list, we can then see the values being received by using `listener <topic_name>` command:  `listener test`

    ![listener_test](img/listener_test.png)

- With this, we can confirm that our uORB topic is successfully receiving the values from the /turtle1/pose topic in the turtlesim node!

---  

### Task 3: **Develop Mavlink message for the custom uORB topics** and use it to **send the uORB message**

Instruction: develop Mavlink message and use it to stream the value from the Test uORB message.
Then, verify that the Mavlink is being streamed by using Wireshark with a custom WLua plugin.

- Install `Wireshark` first on your machine.
- Since PX4 documentation is very good at explaining, try using this [official documentation](https://docs.px4.io/main/en/middleware/mavlink.html).  
- Follow the [Streaming Mavlink Messages](https://docs.px4.io/main/en/middleware/mavlink.html#streaming-mavlink-messages) guide tutorial but instead of using  BATTERY_STATUS_DEMO, create [a new custom mavlink message](https://docs.px4.io/main/en/middleware/mavlink.html#custom-mavlink-messages) to handle the test topic and use it to follow the tutorial.

Note: We're gonna make a Mavlink with the name "POSE_VALUES" which will then receive the pose values from the Test uORB message and stream it.

Step by step:  

1. Open the PX4-Autopilot folder, and copy this `POSE_VALUES` message into the message section of `development.xml` in your PX4 source code, which will be located at:  
`\src\modules\mavlink\mavlink\message_definitions\v1.0\development.xml`  

    ```xml
        <!-- custom mavlink to handle turtlesim's pose uORB message -->
        <message id="45501" name="POSE_VALUES">
            <description>Mavlink test message to handle custom uorb msg</description>
            <field type="uint64_t" name="timestamp">Timestamp</field>
            <field type="float" name="x">x value</field>
            <field type="float" name="y">y value</field>
            <field type="float" name="theta">theta value</field>
            <field type="float" name="linear_velocity">linear velocity</field>
            <field type="float" name="angular_velocity">angular velocity</field>
        </message>

2. Open the PX4-Autopilot and build the PX4 for sitl with `make px4_sitl` command and confirm that the associated message is generated in:  
`/build/px4_sitl_default/mavlink/development/mavlink_pose_values.h`
        ![mavlink_msg_pose_values](img/mavlink_msg_pose_values.png)

3. Open the PX4-Autopilot folder using your preferred IDE, and head over to `/src/modules/mavlink/streams`.

4. Inside the `streams` folder, create a new file named `POSE_VALUES.hpp`

5. Add the headers for the uORB message to the top of the file (the required MAVLink headers should already be available):  
`#include <uORB/topics/test.h>`

6. Copy the streaming class below into the file:

    ```hpp
        class MavlinkStreamPoseValues : public MavlinkStream
        {
        public:
            static MavlinkStream *new_instance(Mavlink *mavlink)
            {
                return new MavlinkStreamPoseValues(mavlink);
            }
            const char *get_name() const
            {
                return MavlinkStreamPoseValues::get_name_static();
            }
            static const char *get_name_static()
            {
                return "POSE_VALUES";
            }
            static uint16_t get_id_static()
            {
                return MAVLINK_MSG_ID_POSE_VALUES;
            }
            uint16_t get_id()
            {
                return get_id_static();
            }
            unsigned get_size()
            {
                return MAVLINK_MSG_ID_POSE_VALUES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
            }

        private:
            // Subscription to a single-instance uORB topic
            uORB::Subscription _pose_values_sub{ORB_ID(test)};

            /* do not allow copying this class */
            MavlinkStreamPoseValues(MavlinkStreamPoseValues &);
            MavlinkStreamPoseValues& operator=(const MavlinkStreamPoseValues &);

        protected:
            explicit MavlinkStreamPoseValues(Mavlink *mavlink) : MavlinkStream(mavlink)
            {}

            bool send() override
            {
                test_s pose_values{};
                if (_pose_values_sub.update(&pose_values))
                {
                    // Define the MAVLink message
                    mavlink_pose_values_t pose_msg{};
                    pose_msg.timestamp = pose_values.timestamp;
                    pose_msg.x = pose_values.x;
                    pose_msg.y = pose_values.y;
                    pose_msg.theta = pose_values.theta;
                    pose_msg.linear_velocity = pose_values.linear_velocity;
                    pose_msg.angular_velocity = pose_values.angular_velocity;

                    // Send the MAVLink message
                    mavlink_msg_pose_values_send_struct(_mavlink->get_channel(), &pose_msg);
                    return true;
                }
                return false;
            }
        };
    ```

    As stated by PX4 official documentation, the public section is 'near boilerplate'. For your own streaming classes these can just be copied and modified to match the values for your MAVLink message.
    `Focus on the protected section`, under the the send(), we define the `pose_values` using our generated uORB struct header, which is `test_s`. More on this are explained on the [Official PX4 Mavlink documentation](https://docs.px4.io/main/en/middleware/mavlink.html#streaming-mavlink-messages)

7. To stream the message by default, `open mavlink_main.cpp`. Navigate to the 'switch case' for **MAVLINK_MODE_NORMAL**, and add the following line within it:
`configure_stream_local("POSE_VALUES", 0.2f);`

    Now we need to create a WLua plugin using our `development.xml` message definition to test our Mavlink using Wireshark. Read PX4 documentation [here](https://docs.px4.io/main/en/middleware/mavlink.html#testing) for more info.

8. Open our Mavlink Toolchain from our PX4-Autopilot folder,  
`PX4-Autopilot/src/modules/mavlink/mavlink`  
Right click on the folder and click on **open in terminal**.
        ![mavlink_px4](img/mavlink_px4.png)

9. As we want to build wireshark's WLua plugin for our development.xml, we can use this command:  

    `python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_development message_definitions/v1.0/development.xml`

    ![mavlink_command](img/mavlink_command.png)  
    - After running the command, you should see a new mavlink_2_development.lua plugin file in the folder.  

    ![mavlink_plugin](img/mavlink_plugin.png)

10. To import the plugin into Wireshark:

    1. Open Wireshark and follow the menu: Help > About Wireshark > Folders
    2. Click on the "Personal Lua Plugin" location to create/open the wireshark Lua plugin's folder.
    3. Copy our `mavlink_2_development.lua` file from the mavlink folder into the wireshark plugin's folder.
    4. Check in Help > About > Wireshark > Plugins. You should find the plugin in the list.

11. Now run every tools and node needed to test the Mavlink:
    1. PX4_sitl
    2. MicroXRCE Agent
    3. QGroundControl
    4. ROS 2 turtlesim_node
    5. posepub node

        ![run_mavlink](img/runmavlink.png)

12. Run the wireshark application, and choose `loopback` as the capture interface.

13. Capture the traffic for about 30 seconds and stop it.

14. On the display filter, use this command:  
`not icmp && mavlink_proto.POSE_VALUES_timestamp`

    ![mavlinkpose_values](img/wiresharkpose_values.png)

15. Click on any of the filtered packet, and click on Mavlink Protocol > Payload:
    ![pose_payload](img/pose_payload.png)

By confirming that the values being streamed are in the correct format, we have verifies that the Mavlink values is being streamed successfully.

### Starting from scratch to install ROS and run the `turtlesim` package in detail with paths explained.

![Turtlesim running](https://github.com/m3hm0o0ud/ros_turtlesim/blob/main/TURTLE.jpg)

### Step 1: Setting Up the ROS Environment

1. **Update the system and ensure root privileges:**
   ```sh
   sudo apt-get update
   sudo apt-get upgrade
   ```

2. **Add ROS keys:**
   ```sh
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   ```

3. **Add ROS repositories:**
   ```sh
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

4. **Install ROS:**
   ```sh
   sudo apt-get update
   sudo apt-get install ros-noetic-desktop-full
   ```

5. **Initialize rosdep:**
   ```sh
   sudo rosdep init
   rosdep update
   ```

6. **Configure the environment:**
   ```sh
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Step 2: Create a Catkin Workspace

1. **Create the workspace directory:**
   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

2. **Configure the environment to include the workspace:**
   ```sh
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Step 3: Install and Use `turtlesim`

1. **Install `turtlesim`:**
   ```sh
   sudo apt-get install ros-noetic-turtlesim
   ```

2. **Create a new ROS package:**
   ```sh
   cd ~/catkin_ws/src
   catkin_create_pkg my_turtle std_msgs rospy roscpp turtlesim
   ```

3. **Build the package:**
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```

4. **Launch `turtlesim_node`:**
   - Open a new terminal and run `roscore`:
   ```sh
   roscore
   ```
   - In another terminal, run `turtlesim_node`:
   ```sh
   rosrun turtlesim turtlesim_node
   ```

### Step 4: Move the Turtle Using Python

1. **Create the turtle mover file:**
   - In the directory `~/catkin_ws/src/my_turtle/src/`, create a new file named `turtle_mover.py`:
   ```sh
   cd ~/catkin_ws/src/my_turtle/src
   nano turtle_mover.py
   ```

   - Paste the following code and save the file:
   ```python
   #!/usr/bin/env python3

   import rospy
   from geometry_msgs.msg import Twist

   def move_turtle():
       rospy.init_node('move_turtle', anonymous=True)
       pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
       rate = rospy.Rate(10)  # 10hz
       vel_msg = Twist()

       while not rospy.is_shutdown():
           vel_msg.linear.x = 2.0  # Move forward
           vel_msg.angular.z = 1.0  # Rotate
           pub.publish(vel_msg)
           rate.sleep()

   if __name__ == '__main__':
       try:
           move_turtle()
       except rospy.ROSInterruptException:
           pass
   ```

2. **Modify `CMakeLists.txt`:**
   - Add the following line to `CMakeLists.txt` in `~/catkin_ws/src/my_turtle/`:
   ```cmake
   catkin_install_python(PROGRAMS src/turtle_mover.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

3. **Rebuild the package:**
   ```sh
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Run the turtle mover file:**
   - In a new terminal, run the file:
   ```sh
   rosrun my_turtle turtle_mover.py
   ```

### Step 5: Running the `turtlesim` Package

1. **Run the `turtlesim` node:**
   - Open a new terminal and run `roscore`:
   ```sh
   roscore
   ```
   - In another terminal, run the `turtlesim` node:
   ```sh
   rosrun turtlesim turtlesim_node
   ```

2. **Run the `turtle_mover.py` file:**
   - In a new terminal, run the file:
   ```sh
   rosrun my_turtle turtle_mover.py
   ```

### Step 6: Controlling the Turtle

You should now see the turtle moving on the screen. You can adjust the linear and angular speed values in the `turtle_mover.py` file to change the turtle's movement.

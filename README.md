# ros_turtlesim

لتثبيت حزمة TurtleSim وإنشاء مشروع مشابه، اتبع الخطوات التالية:

### 1. تثبيت ROS Noetic وTurtleSim
افتح الطرفية وأدخل الأوامر التالية:
```sh
sudo apt update
sudo apt install ros-noetic-turtlesim
```

### 2. إعداد بيئة العمل (Catkin Workspace)
أنشئ مساحة عمل جديدة:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### 3. إنشاء حزمة جديدة
أنشئ حزمة جديدة تسمى `my_turtle`:
```sh
cd ~/catkin_ws/src
catkin_create_pkg my_turtle std_msgs rospy roscpp turtlesim
```

### 4. إنشاء ملف إطلاق (Launch File)
أنشئ مجلد `launch` وملف `turtlesim.launch`:
```sh
mkdir -p ~/catkin_ws/src/my_turtle/launch
nano ~/catkin_ws/src/my_turtle/launch/turtlesim.launch
```

أضف المحتوى التالي:
```xml
<launch>
  <node name="turtlesim" pkg="turtlesim" type="turtlesim_node" output="screen"/>
  <node name="teleop" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
</launch>
```

### 5. إنشاء سكربت لتحريك السلحفاة
أنشئ مجلد `scripts` وملف `move_turtle.py`:
```sh
mkdir -p ~/catkin_ws/src/my_turtle/scripts
nano ~/catkin_ws/src/my_turtle/scripts/move_turtle.py
```

أضف المحتوى التالي:
```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_turtle():
    rospy.init_node('move_turtle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    
    move_cmd = Twist()
    move_cmd.linear.x = 2.0
    move_cmd.angular.z = 1.0

    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass
```

اجعل السكربت قابلاً للتنفيذ:
```sh
chmod +x ~/catkin_ws/src/my_turtle/scripts/move_turtle.py
```

### 6. تعديل ملفات `CMakeLists.txt` و `package.xml`
افتح الملف `CMakeLists.txt` وأضف التالي:
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_turtle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  turtlesim
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs turtlesim
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

افتح الملف `package.xml` وتأكد من احتوائه على التالي:
```xml
<package format="2">
  <name>my_turtle</name>
  <version>0.0.0</version>
  <description>The my_turtle package</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
  <depend>turtlesim</depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>turtlesim</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>turtlesim</exec_depend>
</package>
```

### 7. بناء الحزمة
انتقل إلى مجلد العمل وابنِ الحزمة:
```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 8. تشغيل المشروع
شغل ملف الإطلاق:
```sh
roslaunch my_turtle turtlesim.launch
```

ثم شغل سكربت `move_turtle.py`:
```sh
rosrun my_turtle move_turtle.py
```

سنبدأ من البداية لتثبيت ROS وتشغيل حزمة `turtlesim` بالتفصيل مع توضيح المسارات.

### الخطوة 1: إعداد بيئة ROS

1. **تحديث نظام التشغيل والتأكد من صلاحيات الروت:**
   افتح الطرفية وقم بتنفيذ الأوامر التالية لتحديث النظام:
   ```sh
   sudo apt-get update
   sudo apt-get upgrade
   ```

2. **إضافة مفاتيح ROS:**
   قم بإضافة مفتاح ROS باستخدام الأمر:
   ```sh
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   ```

3. **إضافة مستودعات ROS:**
   أضف مستودعات ROS إلى النظام:
   ```sh
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

4. **تثبيت ROS:**
   بعد إضافة المستودعات، قم بتثبيت ROS:
   ```sh
   sudo apt-get update
   sudo apt-get install ros-noetic-desktop-full
   ```

5. **تهيئة rosdep:**
   قم بتهيئة rosdep لتحميل التبعيات اللازمة:
   ```sh
   sudo rosdep init
   rosdep update
   ```

6. **تهيئة البيئة:**
   أضف ROS إلى المسار البيئي الخاص بك:
   ```sh
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### الخطوة 2: إنشاء مساحة عمل كاتكين (Catkin Workspace)

1. **إنشاء مساحة عمل كاتكين:**
   قم بإنشاء الدليل الخاص بمساحة العمل:
   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

2. **تهيئة البيئة لتشمل مساحة العمل:**
   أضف مساحة العمل الجديدة إلى المسار البيئي:
   ```sh
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### الخطوة 3: تثبيت واستخدام `turtlesim`

1. **تثبيت `turtlesim`:**
   قم بتثبيت الحزمة باستخدام apt-get:
   ```sh
   sudo apt-get install ros-noetic-turtlesim
   ```

2. **إنشاء حزمة ROS جديدة:**
   انتقل إلى مساحة العمل الخاصة بك وأنشئ حزمة جديدة:
   ```sh
   cd ~/catkin_ws/src
   catkin_create_pkg my_turtle std_msgs rospy roscpp turtlesim
   ```

3. **بناء الحزمة:**
   بعد إنشاء الحزمة، قم ببنائها باستخدام:
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```

4. **إطلاق `turtlesim_node`:**
   افتح طرفية جديدة وشغل roscore:
   ```sh
   roscore
   ```
   في طرفية أخرى، شغل `turtlesim_node`:
   ```sh
   rosrun turtlesim turtlesim_node
   ```

### الخطوة 4: تحريك السلحفاة باستخدام Python

1. **إنشاء ملف تحريك السلحفاة:**
   في الدليل `~/catkin_ws/src/my_turtle/src/`، أنشئ ملفًا جديدًا باسم `turtle_mover.py`:
   ```sh
   cd ~/catkin_ws/src/my_turtle/src
   nano turtle_mover.py
   ```
   الصق الكود التالي واحفظ الملف:
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

2. **تعديل CMakeLists.txt:**
   أضف السطر التالي إلى `CMakeLists.txt` في `~/catkin_ws/src/my_turtle/`:
   ```cmake
   catkin_install_python(PROGRAMS src/turtle_mover.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

3. **إعادة بناء الحزمة:**
   قم بإعادة بناء الحزمة:
   ```sh
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **تشغيل ملف تحريك السلحفاة:**
   في طرفية جديدة، شغل الملف:
   ```sh
   rosrun my_turtle turtle_mover.py
   ```

### الخطوة 5: تشغيل حزمة `turtlesim`

1. **تشغيل عقدة `turtlesim`:**
   افتح طرفية جديدة وشغل roscore:
   ```sh
   roscore
   ```
   في طرفية أخرى، شغل عقدة `turtlesim`:
   ```sh
   rosrun turtlesim turtlesim_node
   ```

2. **تشغيل ملف `turtle_mover.py`:**
   في طرفية جديدة، شغل الملف:
   ```sh
   rosrun my_turtle turtle_mover.py
   ```

### الخطوة 6: التحكم بالسلحفاة

يمكنك الآن رؤية السلحفاة تتحرك على الشاشة. يمكنك تعديل قيم السرعة الخطية والزوايا في الملف `turtle_mover.py` لتغيير حركة السلحفاة.

roslaunch chess_detector camera.launch
roslaunch chess_detector split_squares.launch

#rosrun chess_detector aruco_detector.py




Bringup in Gazebo

export GAZEBO_MODEL_PATH=GAZEBO_MODEL_PATH:~/bme_catkin_ws/src/chessboard-detector/gazebo_models/

roslaunch ur_e_gazebo ur3e.launch limited:=true world_file:='$(find chess_detector)/world/chessboard.world' z:=1.02
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true

Bringup real robot:
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=10.0.0.244 limited:=true
roslaunch rh_p12_rn_a_tools bringup.launch

roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch limited:=true
roslaunch ur3_e_moveit_config moveit_rviz.launch config:=true

Start chess:
rosrun ur_tools chess.py


In ROS Melodic we have to compile cv_bridge against Python3:

sudo apt-get install python-catkin-tools python3-dev python3-numpy

mkdir ~/cv_bridge_ws && cd ~/cv_bridge_ws

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

catkin config --install

mkdir src
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git

cd ~/catkin_build_ws
catkin build cv_bridge
source install/setup.bash --extend
source /home/david/cv_bridge_ws/install/setup.bash --extend

```xml
...
      </link>
    </model>
    <!-- A gazebo links attacher -->
    <plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/>
  </world>
</sdf>
```
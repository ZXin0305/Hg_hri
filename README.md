(1)byp80:
BackYard机械爪的控制驱动;
运行：roslaunch byp80 byp80 ip:="你自己设置的爪子的IP"（我的是192.168.58.9）;
这款爪子需要先在IP对应的网址进行标定再使用

(2)camera_wrapper:
将相机与机械臂同时标定到世界坐标系(marker)中;
运行：roslaunch camera_wrapper human.launch;

(3)fr3_description\fr3_moveit_config\ros_control_boilerplate:
法奥机器人所需的驱动，运行之后会启动一个moveit界面，拖动可以控制真实机械臂;
运行：roslaunch fr3_moveit_config demo.launch;
运行这个之后再运行(2)中的指令，就能将相机坐标系\机器人基坐标系\世界坐标系联系起来;

(4)hg_hri:
机器人控制程序，基于ROS平台将多模态手势识别结果\人员姿态信息等发送到机器人控制模块，有信息解码块将获取的信息进行处理，并控制机械臂;
运行：
1) rosrun hg_hri fr3_point  --> 发布预定义的空间点;
2) rosrun hg_hri human_sub  --> 订阅人体信息，包括姿态\左右手势\是否为操作者;
　　　　　　　　　　　　　　　　发布人体空间姿态到RVIZ中,并将其余信息发送到机器人控制块;
3) rosrun hg_hri fr3_robot  --> 通过订阅2)中的信息，控制机器人进行运动，包含多种模式：固定点抓取\末端平移和旋转\路径点记录和重现\急停和运动恢复\爪子开闭等等;

(5)rosparam_shortcuts:
编译(3)时需要的库，如果之前已经安装了就不需要了.

以上代码都是基于ROS/C++的，使用了一些tf转换和关节角度计算等技术，在使用时需要创建自己的工作空间并安装需要的库，之后进行正常编译就可以使用．注意：如果使用其他机械臂，仅需要更改相应的几个控制函数就行．例如：sendCmd([Args...]).....



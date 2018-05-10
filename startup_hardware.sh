#!/bin/bash

#xterm -hold -e "source ~/ws_irp6/mlotz/devel/setup.bash" &
#sleep 5s
#xterm -hold -e "source ~/ws_irp6/mlotz/devel/setup.bash ; roslaunch irp6_bringup irp6-p-nohardware.launch" &
#sleep 10s
#xterm -hold -e "source ~/ws_irp6/mlotz/devel/setup.bash ; rqt" &
#xterm -hold -e "source ~/ws_irp6/mlotz/devel/setup.bash ; rviz rviz" &

gnome-terminal -x bash -c 'cd ~/msc2_ws_irp6/; source /opt/ws_irp6/setup.bash; catkin_make; bash' &
gnome-terminal -x bash -c 'ssh -t gerwazy "source msc2_ws_irp6/devel/setup.bash; roslaunch irp6_bringup irp6-common-realclock.launch; bash"' &
sleep 15s
gnome-terminal -x bash -c 'ssh -t gerwazy "source msc2_ws_irp6/devel/setup.bash; roslaunch msc_mlotz_pkg repo_irp6-p-hardware.launch; bash"' &
gnome-terminal -x bash -c 'source ~/msc2_ws_irp6/devel/setup.bash; export ROS_MASTER_URI=http://gerwazy:11311; export ROS_IP=192.168.18.13; rqt; bash' &
gnome-terminal -x bash -c 'source ~/msc2_ws_irp6/devel/setup.bash; export ROS_MASTER_URI=http://gerwazy:11311; export ROS_IP=192.168.18.13; rviz rviz; bash' &
gnome-terminal -x bash -c 'source ~/msc2_ws_irp6/devel/setup.bash; export ROS_MASTER_URI=http://gerwazy:11311; export ROS_IP=192.168.18.13; rostopic echo /irp6p_tfg/joint_state; bash' &
gnome-terminal -x bash -c 'source ~/msc2_ws_irp6/devel/setup.bash; export ROS_MASTER_URI=http://gerwazy:11311; export ROS_IP=192.168.18.13; echo "overScript command terminal::"; bash' &
gnome-terminal -x bash -c 'source ~/msc2_ws_irp6/devel/setup.bash; export ROS_MASTER_URI=http://gerwazy:11311; export ROS_IP=192.168.18.13; rostopic echo /irp6p_arm/cartesian_position; bash' &
gnome-terminal -x bash -c 'source ~/msc2_ws_irp6/devel/setup.bash; export ROS_MASTER_URI=http://gerwazy:11311; export ROS_IP=192.168.18.13; rostopic echo /optoCompensator1/force_out; bash' &
gnome-terminal -x bash -c 'source ~/msc2_ws_irp6/devel/setup.bash; export ROS_MASTER_URI=http://gerwazy:11311; export ROS_IP=192.168.18.13; rostopic echo /optoCompensator2/force_out; bash' &





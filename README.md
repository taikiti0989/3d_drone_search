# 3d_drone_search
静的三次元探索ドローン

1: roslaunch bebop_teleop bebop_connection.launch

2: roslaunch bebop_teleop teleop.launch

3: roslaunch darknet_ros darknet_ros.launch

4: rosrun kl_evaluation kl_evaluation

5: rosrun keyboard keyboard    ⇨ 1を押す

6: roslaunch orb_slam2_ros orb_slam2_bebop.launch

7: roslaunch ar_track_alvar ar_orb_bebop.launch 
 
8: roslaunch orb_pose_republisher orb_pose_republisher.launch

9: rosrun gaussian_py gaussian.py

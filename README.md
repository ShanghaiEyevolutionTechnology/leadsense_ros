# ROS Package For LeadSense Stereo Camera

## Quick Start 

1. Create a workspace named **catkin_ws** under your HOME folder following the ros [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

2. Open a terminal, download and build the package:
```
cd ~/catkin_ws/src
git clone https://github.com/ShanghaiEyevolutionTechnology/leadsense_ros.git
cd ../
catkin_make
echo source $(pwd)/devel/setup.bash >> ~/.bashrc
source ~/.bashrc
```
3. Start the leadsense camera:
```
roslaunch leadsense_ros leadsense_open.launch 
```

### Rectified Image Topics:
- /leadsense/left/image_rect_gray   
- /leadsense/right/image_rect_gray   
- /leadsense/depth/depth_registered

### PointCloud Topic：
- /leadsense/point_cloud/cloud_registered    
	
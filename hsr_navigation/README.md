# navigation
The navigation repository. Containing everything navigation related that is not forked from other sources.

# Quickstart
In order to launch the correct map and the snap map icp node, use this launch file: 
	
	roslaunch hsr_navigation roslaunch hsr_navigation hsr_map_and_snap_map.launch


# Navigation Doc

### setup
The following will explain what needs to be set up in order to localize the robot. This order of the steps should be maintained, since it workd out best this way. 

#### Map Server setup
It should come with kinetic, so you only need to pull this repo for the map and do: 

    rosrun map_server map_server hsr_lab_map7.yaml

#### Snap Map ICP
Pull: https://github.com/Suturo1819/snap_map_icp and: 

    roslaunch snap_map_icp snap_map_icp.launch 
    
This tool is important. Otherwise the robot's assumed location will drift over time and the location within the map will get worse and worse the more the robot is moving. This tool prevents that from happening.

#### Rviz
Launch your favorite rviz configuration for the robot. For navigation (if you want to test it via Rviz) it is important that the *tool properties* are set correctly. Right click on the toolbar in rviz, and select *tool properties*. The following window should pop up and the topics should be set accordingly. These are NOT the default topics that Rviz usually has. So if nothing is happening, check if these are set to the following: 

![](https://hackmd.informatik.uni-bremen.de/uploads/upload_8ecbe08daf21b9271d8d3d12a22e6d89.png)

Click the *2D Pose Estimate* button in the upper toolbar in Rviz and click and drag on the map in rviz to set the robots location. If you do that after launching snap_map_icp, the localization will work  better, since it will help to rotate the robot into the right configuration in rviz and snap the laser scan to the walls of the map the robot currently sees. 

#### Mapping
If you want to record a new map, you have to kill a specific node on the robot first:

	rosnode kill /pose_integrator
	
Afterwards, run an instance of rviz so that you can see the map, and execute the following: 

	rosrun hector_mapping hector_mapping _map_size:=2048 _map_resolution:=0.03 _pub_map_odom_transform:=true _scan_topic:=/hsrb/base_scan _use_tf_scan_transformation:=true _map_update_angle_thresh:=2.0 _map_update_distance_thresh:=0.10 _scan_subscriber_queue_size:=1 _update_factor_free:=0.39 _update_factor_occupied:=0.85 _base_frame:=base_link
	
	
Drive the robot around until you are happy with the map. If you end up havinga bad loopclosure, just restart the process.
Once you have a map you are happy with, save it with ros map saver:

	rosrun map_server map_saver NAME
	
Then put it into the location where you want it (usually your_ws/src/hsr_navigation/map) and change the name accordingly in the map.launch file, or launch the map the same way it is described above.

#### HSR Navigation
issue: unstable localization
solution: amcl
reason: apperently the default localization of HSR does not use laser scanner much and relies on odometry most of the time. Using amcl with pr2 parameters solves this issue completely.

#### How to launch better localization (AMCL)
1. Launch apartment-lab-map

```bash
roslaunch hsr_navigation hsr_map.launch
```
2. kill default localizer
```bash
rosnode kill laser_2d_localizer
```
3. launch amcl
```bash
roslaunch hsr_navigation hsr_amcl.launch
```
4. localize robot in rviz and proceed as usual



#### Troubleshooting
if the robot has some frame issues even though you are publishing the map, restart the robot so that the node we killed previously will be restarted.

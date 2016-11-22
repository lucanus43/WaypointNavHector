# WaypointNavHector
Waypoint navigation using Hector Quadrotor
Edit 6:50 PM

Commands:
roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" rviz:=false rtabmapviz:=true rgb_topic:=/camera/rgb/image_raw depth_registered_topic:=/camera/depth/image_raw

roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" rviz:=false rtabmapviz:=true rgb_topic:=/camera/rgb/image_raw depth__topic:=/camera/depth/image_raw wait_for_transform:=1.0


rosrun pcl_ros pointcloud_to_pcd input:=rtabmap/cloud_map


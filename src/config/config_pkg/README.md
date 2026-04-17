# How to start the vehicle.

## Start the system
The autonomy part is per default set to false. So autonomy only launched in vehicle_control.launch.py iff autonomy:=true

### copy-paste autonomy FALSE
´´´bash
ros2 launch config_pkg start_system.launch.py
´´´

### copy-paste autonomy TRUE
´´´bash
ros2 launch config_pkg start_system.launch.py autonomy:=true
´´´
That starts the autonomy process but nav2 is not active yet. We then then we are ready call

´´´bash
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
´´´
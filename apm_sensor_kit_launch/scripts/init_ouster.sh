#!/bin/bash

echo "Starting lidar..."
ros2 service call /sensing/lidar/ouster_driver/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /sensing/lidar/ouster_driver/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
echo "Starting lidar success!"

exit 0
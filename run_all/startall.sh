#!/bin/bash

# ============================================
# Fixed startall.sh - SLAM mapping with real LiDAR
# Key fixes:
#   1. use_sim_time:=false (real hardware, no /clock)
#   2. base_frame corrected to base_link
#   3. slam_params_file points to our fixed config
# ============================================

# Terminal 1: LiDAR driver + TF + RViz (same as before, this is fine)
gnome-terminal --tab --title="leida" -- bash -c "
source /opt/ros/humble/setup.bash
sudo chmod 666 /dev/ttyACM0
cd /home/sunrise/Ros_leida/
source install/setup.bash
ros2 launch lidar_pkg monitor.launch.py
"

# Wait for lidar to initialize
sleep 3

# Terminal 2: Static Odom (fallback when encoder not connected)
#   Publishes odom->base_link TF and /odom so slam_toolbox won't choke
#   If you later connect the encoder, just kill this and run:
#   python3 /home/sunrise/car/Seril_Enconder_noninteractive.py <port> <baud>
gnome-terminal --tab --title="Odom" -- bash -c "
source /opt/ros/humble/setup.bash
cd /home/sunrise/car/
python3 Seril_Enconder_noninteractive.py
python3 Seril_Enconder.py
"

sleep 3

# Terminal 3: SLAM (FIXED)
#   - use_sim_time:=false  -- real hardware clock
#   - slam_params_file -- our corrected config with base_frame: base_link
gnome-terminal --tab --title="slam" -- bash -c "
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_sync_launch.py \
    use_sim_time:=false \
    slam_params_file:=/home/sunrise/run_all/config/mapper_params_online_sync.yaml
"

# Terminal 4: Car control
gnome-terminal --tab --title="car_used" -- bash -c "
cd /home/sunrise/car
python3 Car_key.py
"

#!/bin/bash

# ----------------------------
# Usage check
# ----------------------------
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <JACKAL_IP> <HOST_IP>"
    exit 1
fi

JACKAL_IP="$1"
HOST_IP="$2"

# Workspaces to source
WORKSPACES=(
    "$HOME/ros-humble-ros1-bridge/install/local_setup.bash"
    "$HOME/jackal_ws/install/setup.bash"
    "$HOME/colcon_ws/install/setup.bash"
)

# Function to source workspaces
source_workspaces() {
    for ws in "${WORKSPACES[@]}"; do
        if [ -f "$ws" ]; then
            source "$ws"
        fi
    done
}

# ----------------------------
# Update /etc/hosts
# ----------------------------
HOST_ENTRY="ubuntu $JACKAL_IP"
if ! grep -q "$HOST_ENTRY" /etc/hosts; then
    echo "Adding '$HOST_ENTRY' to /etc/hosts"
    sudo bash -c "echo '$HOST_ENTRY' >> /etc/hosts"
else
    echo "/etc/hosts already has the entry '$HOST_ENTRY'"
fi

# ----------------------------
# Terminal 1: SSH into Jackal
# ----------------------------
gnome-terminal -- bash -c "
    echo 'Opening SSH session...';
    ssh adrian@$JACKAL_IP;
    exec bash
"

sleep 3

# ----------------------------
# Terminal 2: ROS1-ROS2 bridge
# ----------------------------
gnome-terminal -- bash -c "
    echo 'Launching ROS1-ROS2 bridge...';
    $(declare -f source_workspaces)
    source_workspaces
    export ROS_MASTER_URI=http://$JACKAL_IP:11311
    export ROS_IP=$HOST_IP
    ros2 run ros1_bridge dynamic_bridge
    exec bash
"

sleep 5

# ----------------------------
# Terminal 3: Vicon bridge
# ----------------------------
gnome-terminal -- bash -c "
    echo 'Launching Vicon bridge...';
    $(declare -f source_workspaces)
    source_workspaces
    ros2 launch vicon_bridge all_segments.launch.py
    exec bash
"

sleep 10

# ----------------------------
# Terminal 4: Jackal circular motion
# ----------------------------
gnome-terminal -- bash -c "
    echo 'Running circular motion...';
    $(declare -f source_workspaces)
    source_workspaces
    ros2 run jackal_control circle --radius 0.3
    exec bash
"

sleep 1

# ----------------------------
# Terminal 5: Trajectory plotter
# ----------------------------
gnome-terminal -- bash -c "
    echo 'Launching trajectory plotter...';
    $(declare -f source_workspaces)
    source_workspaces
    ros2 run jackal_control traj_plotter --enable_odom --enable_vicon
    exec bash
"

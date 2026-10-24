#!/bin/bash

# Unified launcher script for articubot_one with all components
# Usage: ./launch_all.sh

SESSION_NAME="articubot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-$SCRIPT_DIR}"

# Kill any existing tmux session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create a new tmux session with 2 windows
tmux new-session -d -s $SESSION_NAME -x 200 -y 50

# Window 0: Full robot stack (simulation + map + controllers)
tmux send-keys -t $SESSION_NAME:0 "cd $ROS2_WS && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch articubot_one bringup_all.launch.py" Enter
tmux rename-window -t $SESSION_NAME:0 "robot"

# Window 1: RViz visualization
tmux new-window -t $SESSION_NAME:1
tmux send-keys -t $SESSION_NAME:1 "sleep 3 && cd $ROS2_WS && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch articubot_one rviz_view.launch.py" Enter
tmux rename-window -t $SESSION_NAME:1 "rviz"

# Print session info
echo "=========================================="
echo "Articubot launch started!"
echo "=========================================="
echo ""
echo "ROS2 Workspace: $ROS2_WS"
echo "Tmux Session: $SESSION_NAME"
echo ""
echo "Windows:"
echo "  1. robot  - Full robot stack (simulation + map + controllers)"
echo "  2. rviz   - RViz visualization"
echo ""
echo "To attach to tmux session:"
echo "  tmux attach-session -t $SESSION_NAME"
echo ""
echo "To switch windows (when attached):"
echo "  Ctrl+B then 0 -> robot window"
echo "  Ctrl+B then 1 -> rviz window"
echo ""
echo "To kill the session:"
echo "  tmux kill-session -t $SESSION_NAME"
echo "=========================================="

# Attach to the session
tmux attach-session -t $SESSION_NAME

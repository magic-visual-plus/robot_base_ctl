#!/bin/bash

set -euo pipefail

SESSION_NAME="torso_follow"
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"
MSG_WS_SETUP="/opt/project/robot_base_ctl/ros2_msg/install/setup.bash"
WS_DIR="/opt/project/robot_base_ctl/tosor_ctl"
TORSO_WS_SETUP="$WS_DIR/install/setup.bash"

RMW_IMPLEMENTATION_DEFAULT="rmw_zenoh_cpp"
ROS_DOMAIN_ID_DEFAULT="0"
ROS_LOCALHOST_ONLY_DEFAULT="0"
LOG_DIR="$WS_DIR/logs"
TS="$(date +%Y%m%d_%H%M%S)"
TORSO_LOG="$LOG_DIR/torso_node_${TS}.log"
FOLLOW_LOG="$LOG_DIR/follow_node_${TS}.log"
TOOLS_LOG="$LOG_DIR/tools_${TS}.log"
ZOH_FB_LOG="$LOG_DIR/zoh_fb_cmd_${TS}.log"

CMD_PREP_BASE="source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION_DEFAULT && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_DEFAULT && export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY_DEFAULT && export ZENOH_CONFIG_OVERRIDE=\"mode=\\\"client\\\";connect/endpoints=[\\\"tcp/192.168.1.100:7447\\\"]\" && source \"$MSG_WS_SETUP\""
CMD_PREP_TORSO="$CMD_PREP_BASE && cd \"$WS_DIR\" && ([ -f \"$TORSO_WS_SETUP\" ] || colcon build --packages-select phi_motion_torso) && source \"$TORSO_WS_SETUP\""

CMD_TORSO="$CMD_PREP_TORSO && ros2 run phi_motion_torso torso_node.py --ros-args -p control_mode:=topic 2>&1 | tee -a \"$TORSO_LOG\""
CMD_FOLLOW="$CMD_PREP_TORSO && ros2 run phi_motion_torso torso_teleop_follow_topic_node.py 2>&1 | tee -a \"$FOLLOW_LOG\""
CMD_TOOLS="$CMD_PREP_TORSO && ros2 topic hz /phi/motion/torso/target_height_cmd 2>&1 | tee -a \"$TOOLS_LOG\" && ros2 topic hz /phi/motion/torso/status 2>&1 | tee -a \"$TOOLS_LOG\""
CMD_ZOH_FB="$CMD_PREP_BASE && /usr/bin/python /opt/project/robot_base_ctl/base_ctl/zoh_fb_cmd.py 2>&1 | tee -a \"$ZOH_FB_LOG\""

start() {
    local session_exists=0
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        session_exists=1
    fi

    if [ ! -f "$ROS_DISTRO_SETUP" ]; then
        echo "ROS2 setup not found: $ROS_DISTRO_SETUP"
        exit 1
    fi

    if [ ! -d "$WS_DIR" ]; then
        echo "Workspace not found: $WS_DIR"
        exit 1
    fi
    mkdir -p "$LOG_DIR"

    if [ ! -f "$MSG_WS_SETUP" ]; then
        echo "Message workspace setup not found: $MSG_WS_SETUP"
        echo "Please build message workspace first:"
        echo "  cd /opt/project/robot_base_ctl/ros2_msg"
        echo "  source /opt/ros/humble/setup.bash"
        echo "  colcon build --packages-select pymbc_msgs"
        exit 1
    fi

    if [ "$session_exists" -eq 1 ]; then
        # Avoid duplicate execution: only start missing windows.
        window_exists() {
            local window_name="$1"
            tmux list-windows -t "$SESSION_NAME" -F '#{window_name}' 2>/dev/null | grep -qx "$window_name"
        }

        if ! window_exists "torso_node"; then
            tmux new-window -t "$SESSION_NAME" -n "torso_node" "bash -lc '$CMD_TORSO'"
        fi
        if ! window_exists "follow_node"; then
            tmux new-window -t "$SESSION_NAME" -n "follow_node" "bash -lc 'sleep 2; $CMD_FOLLOW'"
        fi
        if ! window_exists "tools"; then
            tmux new-window -t "$SESSION_NAME" -n "tools" "bash -lc '$CMD_TOOLS'"
        fi
        if ! window_exists "zoh_fb_cmd"; then
            tmux new-window -t "$SESSION_NAME" -n "zoh_fb_cmd" "bash -lc '$CMD_ZOH_FB'"
        fi

        echo "Session '$SESSION_NAME' already exists. Ensured missing windows are running."
        exit 0
    fi

    tmux new-session -d -s "$SESSION_NAME" -n "torso_node"
    tmux send-keys -t "$SESSION_NAME":0 "bash -lc '$CMD_TORSO'" C-m

    tmux new-window -t "$SESSION_NAME" -n "follow_node"
    tmux send-keys -t "$SESSION_NAME":1 "bash -lc 'sleep 2; $CMD_FOLLOW'" C-m

    tmux new-window -t "$SESSION_NAME" -n "tools"
    tmux send-keys -t "$SESSION_NAME":2 "bash -lc '$CMD_TOOLS'" C-m

    tmux new-window -t "$SESSION_NAME" -n "zoh_fb_cmd"
    tmux send-keys -t "$SESSION_NAME":3 "bash -lc '$CMD_ZOH_FB'" C-m

    tmux select-window -t "$SESSION_NAME":0
    echo "Session '$SESSION_NAME' started."
    echo "Windows:"
    echo "  0) torso_node  (topic control mode)"
    echo "  1) follow_node (whole_body -> target_height_cmd)"
    echo "  2) tools       (topic rate monitor)"
    echo "  3) zoh_fb_cmd  (whole_body -> /ref_pose)"
    echo ""
    echo "Log files:"
    echo "  $TORSO_LOG"
    echo "  $FOLLOW_LOG"
    echo "  $TOOLS_LOG"
    echo "  $ZOH_FB_LOG"
    echo ""
    echo "Use:"
    echo "  ./start_torso_follow.sh attach"
    echo "  ./start_torso_follow.sh stop"
}

stop() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        tmux kill-session -t "$SESSION_NAME"
        echo "Session '$SESSION_NAME' stopped."
    else
        echo "Session '$SESSION_NAME' does not exist."
    fi
}

attach() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        tmux attach -t "$SESSION_NAME"
    else
        echo "Session '$SESSION_NAME' does not exist. Run './start_torso_follow.sh start' first."
    fi
}

status() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        tmux list-windows -t "$SESSION_NAME"
    else
        echo "Session '$SESSION_NAME' does not exist."
    fi
}

case "${1:-}" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    attach)
        attach
        ;;
    status)
        status
        ;;
    *)
        echo "Usage: $0 {start|stop|attach|status}"
        exit 1
        ;;
esac


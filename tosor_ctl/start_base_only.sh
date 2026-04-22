#!/bin/bash
#
# 仅底盘：订阅 whole_body -> 发布 /ref_pose（与 start_torso_follow.sh 同套 ROS2 / 消息环境，不含躯干节点）
#

set -euo pipefail

SESSION_NAME="base_only"
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"
MSG_WS_SETUP="/opt/project/robot_base_ctl/ros2_msg/install/setup.bash"
ZOH_FB_CMD="/opt/project/robot_base_ctl/base_ctl/zoh_fb_cmd.py"

RMW_IMPLEMENTATION_DEFAULT="rmw_zenoh_cpp"
ROS_DOMAIN_ID_DEFAULT="0"
ROS_LOCALHOST_ONLY_DEFAULT="0"
LOG_DIR="/opt/project/robot_base_ctl/tosor_ctl/logs"
TS="$(date +%Y%m%d_%H%M%S)"
ZOH_LOG="$LOG_DIR/zoh_fb_cmd_${TS}.log"
TOOLS_LOG="$LOG_DIR/base_tools_${TS}.log"

CMD_PREP="source \"$ROS_DISTRO_SETUP\" && export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-$RMW_IMPLEMENTATION_DEFAULT} && export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT} && export ROS_LOCALHOST_ONLY=\${ROS_LOCALHOST_ONLY:-$ROS_LOCALHOST_ONLY_DEFAULT} && if [ -n \"\${ZENOH_CONFIG_OVERRIDE:-}\" ]; then export ZENOH_CONFIG_OVERRIDE; fi && source \"$MSG_WS_SETUP\""
CMD_ZOH_FB="$CMD_PREP && python3 \"$ZOH_FB_CMD\" 2>&1 | tee -a \"$ZOH_LOG\""
CMD_TOOLS="$CMD_PREP && ros2 topic hz /ref_pose 2>&1 | tee -a \"$TOOLS_LOG\""

start() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "Session '$SESSION_NAME' already exists. Attaching..."
        tmux attach -t "$SESSION_NAME"
        exit 0
    fi

    if [ ! -f "$ROS_DISTRO_SETUP" ]; then
        echo "ROS2 setup not found: $ROS_DISTRO_SETUP"
        exit 1
    fi

    if [ ! -f "$ZOH_FB_CMD" ]; then
        echo "Script not found: $ZOH_FB_CMD"
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

    tmux new-session -d -s "$SESSION_NAME" -n "zoh_fb_cmd"
    tmux send-keys -t "$SESSION_NAME":0 "bash -lc '$CMD_ZOH_FB'" C-m

    tmux new-window -t "$SESSION_NAME" -n "tools"
    tmux send-keys -t "$SESSION_NAME":1 "bash -lc '$CMD_TOOLS'" C-m

    tmux select-window -t "$SESSION_NAME":0
    echo "Session '$SESSION_NAME' started (chassis only)."
    echo "Windows:"
    echo "  0) zoh_fb_cmd  (/phi/motion/teleop/whole_body -> /ref_pose)"
    echo "  1) tools       (ros2 topic hz /ref_pose)"
    echo ""
    echo "Log files:"
    echo "  $ZOH_LOG"
    echo "  $TOOLS_LOG"
    echo ""
    echo "Use:"
    echo "  ./start_base_only.sh attach"
    echo "  ./start_base_only.sh stop"
    echo ""
    echo "Note: closed-loop /cmd_vel needs zoh_rev + bridge separately (e.g. robot_base_ctl/start.sh)."
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
        echo "Session '$SESSION_NAME' does not exist. Run './start_base_only.sh start' first."
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

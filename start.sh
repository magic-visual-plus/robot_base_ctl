#!/bin/bash

SESSION_NAME="base_ctl"

start() {
    # 检查 session 是否已存在
    if tmux has-session -t $SESSION_NAME 2>/dev/null; then
        echo "Session '$SESSION_NAME' already exists. Attaching..."
        tmux attach -t $SESSION_NAME
        exit 0
    fi

    # 创建新的 tmux session
    tmux new-session -d -s $SESSION_NAME -n "bridge"

    # 在第一个窗口运行 bridge.py
    tmux send-keys -t $SESSION_NAME:0 "python3 /opt/project/robot_base_ctl/motor/moons/bridge.py" C-m

    # 创建第二个窗口运行 zoh_rev.py
    tmux new-window -t $SESSION_NAME -n "zoh_rev"
    tmux send-keys -t $SESSION_NAME:1 "python3 /opt/project/robot_base_ctl/base_ctl/zoh_rev.py" C-m

    # 回到第一个窗口并附加到 session
    tmux select-window -t $SESSION_NAME:0
    echo "Session '$SESSION_NAME' started."
    echo "Use './start.sh stop' to stop the session."
    echo "Use './start.sh attach' to attach to the session."
}

stop() {
    if tmux has-session -t $SESSION_NAME 2>/dev/null; then
        tmux kill-session -t $SESSION_NAME
        echo "Session '$SESSION_NAME' stopped."
    else
        echo "Session '$SESSION_NAME' does not exist."
    fi
}

attach() {
    if tmux has-session -t $SESSION_NAME 2>/dev/null; then
        tmux attach -t $SESSION_NAME
    else
        echo "Session '$SESSION_NAME' does not exist. Run './start.sh start' first."
    fi
}

case "$1" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    attach)
        attach
        ;;
    *)
        echo "Usage: $0 {start|stop|attach}"
        echo ""
        echo "Commands:"
        echo "  start   - Start the tmux session with bridge and zoh_rev"
        echo "  stop    - Stop the tmux session"
        echo "  attach  - Attach to the running tmux session"
        exit 1
        ;;
esac

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 2000
sudo ip link set can0 up
ip -details link show can0

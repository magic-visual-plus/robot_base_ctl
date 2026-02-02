# 常用命令行工具

## can 相关
### 设置 can0开启自动启动

```
sudo cp can0.service /etc/systemd/system/can0.service
sudo systemctl daemon-reload
sudo systemctl enable can0.service
sudo systemctl start can0.service
```

### 查看can状态
```
ip -details link show can0
```

### 查看can总线负载
```
canbusload can0@1000000
```
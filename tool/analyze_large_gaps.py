#!/usr/bin/env python3
"""
深入分析大间隔的原因
"""

import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def analyze_large_gaps(bag_path: str):
    """
    详细分析点云数据的大间隔
    """
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic type
    topic_types = reader.get_all_topics_and_types()
    topic_type = None
    for t in topic_types:
        if t.name == "/livox/points":
            topic_type = t.type
            break
    
    if not topic_type:
        print("Error: /livox/points not found")
        return
    
    msg_type = get_message(topic_type)
    
    timestamps = []
    count = 0
    
    print("Reading point cloud data...")
    while reader.has_next():
        (topic_name, data, receive_time_ns) = reader.read_next()
        
        if topic_name == "/livox/points":
            try:
                msg = deserialize_message(data, msg_type)
                msg_time_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                
                timestamps.append({
                    'index': count,
                    'receive_time': receive_time_ns,
                    'msg_time': msg_time_ns,
                    'frame_id': msg.header.frame_id
                })
                count += 1
                
                if count % 5000 == 0:
                    print(f"  Processed {count} messages...")
            except Exception as e:
                pass
    
    print(f"Total messages: {count}\n")
    
    # 计算间隔
    print("="*80)
    print("分析大间隔（>0.2秒）")
    print("="*80)
    
    large_gaps = []
    
    for i in range(1, len(timestamps)):
        # 使用消息时间戳（硬件时间）
        msg_interval = (timestamps[i]['msg_time'] - timestamps[i-1]['msg_time']) / 1e9
        # 使用接收时间戳（系统时间）
        receive_interval = (timestamps[i]['receive_time'] - timestamps[i-1]['receive_time']) / 1e9
        
        if msg_interval > 0.2:
            large_gaps.append({
                'index': i-1,
                'msg_interval': msg_interval,
                'receive_interval': receive_interval,
                'prev_msg_time': timestamps[i-1]['msg_time'],
                'curr_msg_time': timestamps[i]['msg_time'],
                'prev_recv_time': timestamps[i-1]['receive_time'],
                'curr_recv_time': timestamps[i]['receive_time'],
            })
    
    print(f"\n找到 {len(large_gaps)} 个大间隔\n")
    print(f"{'Index':<8} {'消息间隔(s)':<15} {'接收间隔(s)':<15} {'差异(ms)':<12}")
    print("-" * 80)
    
    for gap in large_gaps[:20]:
        diff = abs(gap['msg_interval'] - gap['receive_interval']) * 1000
        print(f"{gap['index']:<8} {gap['msg_interval']:<15.6f} {gap['receive_interval']:<15.6f} {diff:<12.3f}")
    
    if len(large_gaps) > 20:
        print(f"... 还有 {len(large_gaps)-20} 个")
    
    # 统计分析
    msg_intervals = [g['msg_interval'] for g in large_gaps]
    recv_intervals = [g['receive_interval'] for g in large_gaps]
    
    print(f"\n{'='*80}")
    print("统计分析")
    print(f"{'='*80}")
    print(f"\n基于消息时间戳（硬件时间）:")
    print(f"  平均大间隔: {sum(msg_intervals)/len(msg_intervals):.6f} 秒")
    print(f"  最小: {min(msg_intervals):.6f} 秒")
    print(f"  最大: {max(msg_intervals):.6f} 秒")
    
    print(f"\n基于接收时间戳（系统时间）:")
    print(f"  平均大间隔: {sum(recv_intervals)/len(recv_intervals):.6f} 秒")
    print(f"  最小: {min(recv_intervals):.6f} 秒")
    print(f"  最大: {max(recv_intervals):.6f} 秒")
    
    # 分析是否一致
    print(f"\n{'='*80}")
    print("结论")
    print(f"{'='*80}")
    
    diffs = [abs(g['msg_interval'] - g['receive_interval']) * 1000 for g in large_gaps]
    avg_diff = sum(diffs) / len(diffs)
    
    if avg_diff < 10:  # 小于10ms
        print("\n✓ 消息时间戳和接收时间戳的间隔基本一致（差异<10ms）")
        print("  说明：硬件确实产生了这些大间隔，不是软件问题")
        print("  可能原因：")
        print("    1. 传感器在某些时刻丢帧或跳帧")
        print("    2. 硬件处理延迟（如数据缓冲区满）")
        print("    3. 传感器工作模式变化")
    else:
        print("\n⚠ 消息时间戳和接收时间戳的间隔不一致")
        print(f"  平均差异: {avg_diff:.3f} ms")
        print("  可能原因：")
        print("    1. 消息在发布端被积压")
        print("    2. 网络传输延迟")
        print("    3. rosbag 记录延迟")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_large_gaps.py <bag_path>")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    if bag_path.endswith('.db3'):
        from pathlib import Path
        bag_path = str(Path(bag_path).parent)
    
    analyze_large_gaps(bag_path)

if __name__ == "__main__":
    main()

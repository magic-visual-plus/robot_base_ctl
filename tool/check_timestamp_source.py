#!/usr/bin/env python3
"""
检查时间戳的来源：对比接收时间和消息头时间戳
"""

import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def analyze_timestamp_source(bag_path: str, topic: str, sample_size: int = 100):
    """
    分析时间戳来源，对比接收时间和消息时间戳
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
        if t.name == topic:
            topic_type = t.type
            break
    
    if not topic_type:
        print(f"Error: Topic {topic} not found")
        return
    
    print(f"\n{'='*70}")
    print(f"Analyzing: {topic}")
    print(f"Type: {topic_type}")
    print(f"{'='*70}\n")
    
    msg_type = get_message(topic_type)
    
    samples = []
    count = 0
    
    while reader.has_next() and count < sample_size:
        (topic_name, data, receive_time_ns) = reader.read_next()
        
        if topic_name == topic:
            try:
                msg = deserialize_message(data, msg_type)
                
                if hasattr(msg, 'header'):
                    msg_time_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                    time_diff = (receive_time_ns - msg_time_ns) / 1e6  # 转换为毫秒
                    
                    samples.append({
                        'index': count,
                        'receive_time': receive_time_ns,
                        'msg_time': msg_time_ns,
                        'diff_ms': time_diff
                    })
                    count += 1
            except Exception as e:
                pass
    
    if not samples:
        print("No samples collected")
        return
    
    # 显示前10个样本
    print("Sample data (first 10):")
    print(f"{'Index':<8} {'Receive Time (ns)':<22} {'Msg Time (ns)':<22} {'Diff (ms)':<12}")
    print("-" * 70)
    for s in samples[:10]:
        print(f"{s['index']:<8} {s['receive_time']:<22} {s['msg_time']:<22} {s['diff_ms']:<12.3f}")
    
    # 统计分析
    diffs = [s['diff_ms'] for s in samples]
    avg_diff = sum(diffs) / len(diffs)
    min_diff = min(diffs)
    max_diff = max(diffs)
    
    print(f"\n{'='*70}")
    print("Time Difference Statistics (Receive - Message):")
    print(f"{'='*70}")
    print(f"Average: {avg_diff:.3f} ms")
    print(f"Min: {min_diff:.3f} ms")
    print(f"Max: {max_diff:.3f} ms")
    
    print(f"\n{'='*70}")
    print("Timestamp Source Analysis:")
    print(f"{'='*70}")
    
    # 判断时间戳类型
    if abs(avg_diff) < 1:  # 小于1毫秒
        print("✓ 时间戳非常接近接收时间")
        print("  可能是：驱动节点使用系统时间 (this->now())")
    elif 0 < avg_diff < 100:  # 0-100毫秒之间
        print("✓ 消息时间戳略早于接收时间")
        print("  可能是：硬件时间戳 + 传输延迟")
        print(f"  平均传输延迟: {avg_diff:.3f} ms")
    elif avg_diff > 100:
        print("⚠ 消息时间戳明显早于接收时间")
        print("  可能是：硬件时间戳，或者时钟不同步")
    elif avg_diff < -100:
        print("⚠ 消息时间戳晚于接收时间")
        print("  可能存在：时钟同步问题")
    else:
        print("ℹ 时间差在合理范围内")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_timestamp_source.py <bag_path>")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    if bag_path.endswith('.db3'):
        from pathlib import Path
        bag_path = str(Path(bag_path).parent)
    
    # 分析两个 topic
    analyze_timestamp_source(bag_path, "/livox/imu", sample_size=100)
    analyze_timestamp_source(bag_path, "/livox/points", sample_size=100)

if __name__ == "__main__":
    main()

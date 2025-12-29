#!/usr/bin/env python3
"""
Script to check timestamp alignment between IMU and Point Cloud data in ROS2 bag.
Verifies if all point cloud timestamps fall within IMU timestamp intervals.
Uses rosbag2_py API for better performance and compatibility.
"""

import sys
from typing import List, Tuple
from pathlib import Path

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Error: rosbag2_py not found. Please install it:")
    print("  sudo apt install ros-humble-rosbag2-py")
    print("  or for your ROS2 distro: ros-<distro>-rosbag2-py")
    sys.exit(1)


def get_timestamps_from_bag(bag_path: str, topic: str) -> List[Tuple[int, int]]:
    """
    Extract timestamps from a ROS2 bag topic using rosbag2_py API.
    Returns list of (receive_timestamp, message_timestamp) tuples in nanoseconds.
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
        print(f"Error: Topic {topic} not found in bag")
        return []
    
    print(f"  Reading topic: {topic} (type: {topic_type})")
    
    # Get message type
    msg_type = get_message(topic_type)
    print(f"  Message type: {msg_type}")
    
    timestamps = []
    count = 0
    
    while reader.has_next():
        (topic_name, data, t_ns) = reader.read_next()
        
        if topic_name == topic:
            count += 1
            
            # Deserialize message to get header timestamp
            try:
                msg = deserialize_message(data, msg_type)
                # Get timestamp from message header
                if hasattr(msg, 'header'):
                    msg_timestamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                else:
                    msg_timestamp = t_ns  # Use receive timestamp if no header
                
                timestamps.append((t_ns, msg_timestamp))
            except Exception as e:
                # If deserialization fails, just use receive timestamp
                timestamps.append((t_ns, t_ns))
            
            # Progress indicator
            if count % 10000 == 0:
                print(f"  Processed {count} messages...")
    
    print(f"  Total messages read: {count}")
    return timestamps


def check_alignment(imu_timestamps: List[Tuple[int, int]], 
                   point_timestamps: List[Tuple[int, int]]) -> None:
    """
    Check if all point cloud timestamps fall within IMU timestamp range.
    """
    if not imu_timestamps:
        print("Error: No IMU data found")
        return
    
    if not point_timestamps:
        print("Error: No point cloud data found")
        return
    
    print(f"\n{'='*70}")
    print(f"ROS Bag Timestamp Alignment Check")
    print(f"{'='*70}\n")
    
    # Statistics
    print(f"IMU messages: {len(imu_timestamps)}")
    print(f"Point Cloud messages: {len(point_timestamps)}")
    
    # Get IMU timestamp range (using message timestamps - hardware time)
    imu_start = imu_timestamps[0][1]
    imu_end = imu_timestamps[-1][1]
    imu_duration = (imu_end - imu_start) / 1e9  # seconds
    
    # Get Point Cloud timestamp range (using message timestamps - hardware time)
    point_start = point_timestamps[0][1]
    point_end = point_timestamps[-1][1]
    point_duration = (point_end - point_start) / 1e9  # seconds
    
    print(f"\nIMU timestamp range:")
    print(f"  Start: {imu_start} ns ({imu_start/1e9:.6f} s)")
    print(f"  End:   {imu_end} ns ({imu_end/1e9:.6f} s)")
    print(f"  Duration: {imu_duration:.3f} seconds")
    
    print(f"\nPoint Cloud timestamp range:")
    print(f"  Start: {point_start} ns ({point_start/1e9:.6f} s)")
    print(f"  End:   {point_end} ns ({point_end/1e9:.6f} s)")
    print(f"  Duration: {point_duration:.3f} seconds")
    
    # Check alignment
    print(f"\n{'='*70}")
    print(f"Alignment Check Results")
    print(f"{'='*70}\n")
    
    # Check if all point timestamps are within IMU range
    points_before_imu = 0
    points_after_imu = 0
    points_aligned = 0
    
    misaligned_points = []
    
    for idx, (point_recv_ts, point_msg_ts) in enumerate(point_timestamps):
        # Use message timestamp (hardware time) for alignment check
        if point_msg_ts < imu_start:
            points_before_imu += 1
            misaligned_points.append((idx, point_msg_ts, "before IMU start"))
        elif point_msg_ts > imu_end:
            points_after_imu += 1
            misaligned_points.append((idx, point_msg_ts, "after IMU end"))
        else:
            points_aligned += 1
    
    # Print results
    print(f"✓ Points within IMU range: {points_aligned} ({points_aligned/len(point_timestamps)*100:.2f}%)")
    
    if points_before_imu > 0:
        print(f"✗ Points before IMU start: {points_before_imu} ({points_before_imu/len(point_timestamps)*100:.2f}%)")
    
    if points_after_imu > 0:
        print(f"✗ Points after IMU end: {points_after_imu} ({points_after_imu/len(point_timestamps)*100:.2f}%)")
    
    # Calculate time offset
    time_offset_start = (point_start - imu_start) / 1e9
    time_offset_end = (point_end - imu_end) / 1e9
    
    print(f"\nTime offsets:")
    print(f"  Point start vs IMU start: {time_offset_start:+.6f} seconds")
    print(f"  Point end vs IMU end: {time_offset_end:+.6f} seconds")
    
    # Check frequency alignment
    imu_freq = len(imu_timestamps) / imu_duration if imu_duration > 0 else 0
    point_freq = len(point_timestamps) / point_duration if point_duration > 0 else 0
    
    print(f"\nData frequencies:")
    print(f"  IMU: {imu_freq:.2f} Hz")
    print(f"  Point Cloud: {point_freq:.2f} Hz")
    
    # Expected points based on IMU duration
    expected_points = point_freq * imu_duration
    print(f"\nExpected points in IMU duration: {expected_points:.0f}")
    print(f"Actual points in range: {points_aligned}")
    
    # Check IMU data distribution between point cloud frames
    print(f"\n{'='*70}")
    print(f"IMU Data Distribution Between Point Cloud Frames")
    print(f"{'='*70}\n")
    
    if len(point_timestamps) > 1:
        import bisect
        
        # Extract IMU message timestamps for binary search
        imu_msg_times = [imu_msg_ts for _, imu_msg_ts in imu_timestamps]
        
        imu_counts_between_frames = []
        print("  Calculating IMU distribution...")
        
        for i in range(1, len(point_timestamps)):
            prev_point_time = point_timestamps[i-1][1]
            curr_point_time = point_timestamps[i][1]
            
            # Use binary search for efficiency
            left_idx = bisect.bisect_left(imu_msg_times, prev_point_time)
            right_idx = bisect.bisect_left(imu_msg_times, curr_point_time)
            imu_count = right_idx - left_idx
            
            imu_counts_between_frames.append({
                'index': i-1,
                'count': imu_count,
                'prev_time': prev_point_time,
                'curr_time': curr_point_time,
                'interval': (curr_point_time - prev_point_time) / 1e9
            })
            
            if i % 5000 == 0:
                print(f"    Processed {i}/{len(point_timestamps)-1} intervals...")
        
        if imu_counts_between_frames:
            avg_imu_count = sum(item['count'] for item in imu_counts_between_frames) / len(imu_counts_between_frames)
            min_imu_item = min(imu_counts_between_frames, key=lambda x: x['count'])
            max_imu_item = max(imu_counts_between_frames, key=lambda x: x['count'])
            
            print(f"\nAverage IMU messages per point cloud interval: {avg_imu_count:.2f}")
            print(f"\nMinimum IMU count: {min_imu_item['count']}")
            print(f"  Between point {min_imu_item['index']} and {min_imu_item['index']+1}")
            print(f"  Time interval: {min_imu_item['interval']:.6f} seconds")
            print(f"  Point timestamps: {min_imu_item['prev_time']} -> {min_imu_item['curr_time']}")
            
            print(f"\nMaximum IMU count: {max_imu_item['count']}")
            print(f"  Between point {max_imu_item['index']} and {max_imu_item['index']+1}")
            print(f"  Time interval: {max_imu_item['interval']:.6f} seconds")
            print(f"  Point timestamps: {max_imu_item['prev_time']} -> {max_imu_item['curr_time']}")
            
            # Show distribution
            zero_imu = len([item for item in imu_counts_between_frames if item['count'] == 0])
            if zero_imu > 0:
                print(f"\n⚠ Warning: {zero_imu} intervals have 0 IMU messages")
                print(f"\nZero IMU intervals details:")
                zero_items = [item for item in imu_counts_between_frames if item['count'] == 0]
                for item in zero_items[:10]:  # Show first 10
                    print(f"  Between point {item['index']} and {item['index']+1}:")
                    print(f"    Time interval: {item['interval']:.6f} seconds")
                    print(f"    Timestamps: {item['prev_time']} -> {item['curr_time']}")
                    
                    # Debug: check nearby IMU data
                    left_idx = bisect.bisect_left(imu_msg_times, item['prev_time'])
                    right_idx = bisect.bisect_left(imu_msg_times, item['curr_time'])
                    print(f"    Binary search indices: left={left_idx}, right={right_idx}")
                    
                    # Show nearby IMU timestamps
                    if left_idx > 0:
                        print(f"    IMU before interval: {imu_msg_times[left_idx-1]} (index {left_idx-1})")
                    if left_idx < len(imu_msg_times):
                        print(f"    IMU at/after start: {imu_msg_times[left_idx]} (index {left_idx})")
                    if right_idx < len(imu_msg_times):
                        print(f"    IMU at/after end: {imu_msg_times[right_idx]} (index {right_idx})")
                    
                if len(zero_items) > 10:
                    print(f"  ... and {len(zero_items)-10} more zero IMU intervals")
    
    # Check for gaps in point cloud data
    print(f"\n{'='*70}")
    print(f"Gap Analysis")
    print(f"{'='*70}\n")
    
    if len(point_timestamps) > 1:
        time_diffs = []
        for i in range(1, len(point_timestamps)):
            # Use message timestamp (hardware time), not receive timestamp
            diff = (point_timestamps[i][1] - point_timestamps[i-1][1]) / 1e9
            time_diffs.append(diff)
        
        avg_diff = sum(time_diffs) / len(time_diffs)
        max_diff = max(time_diffs)
        min_diff = min(time_diffs)
        
        print(f"Point cloud interval statistics:")
        print(f"  Average: {avg_diff:.6f} seconds ({1/avg_diff:.2f} Hz)")
        print(f"  Min: {min_diff:.6f} seconds")
        print(f"  Max: {max_diff:.6f} seconds")
        
        # Find large gaps (> 2x average)
        large_gaps = [(i, diff) for i, diff in enumerate(time_diffs) if diff > 2 * avg_diff]
        if large_gaps:
            print(f"\n⚠ Found {len(large_gaps)} large gaps (>2x average interval):")
            for idx, gap in large_gaps[:10]:  # Show first 10
                print(f"  Between point {idx} and {idx+1}: {gap:.6f} seconds")
            if len(large_gaps) > 10:
                print(f"  ... and {len(large_gaps)-10} more")
        
        # Find small intervals (< average)
        small_intervals = [(i, diff) for i, diff in enumerate(time_diffs) if diff < avg_diff]
        if small_intervals:
            print(f"\n⚠ Found {len(small_intervals)} small intervals (<average interval):")
            # Show first 10 smallest
            sorted_small = sorted(small_intervals, key=lambda x: x[1])
            for idx, interval in sorted_small[:10]:
                print(f"  Between point {idx} and {idx+1}: {interval:.6f} seconds")
            if len(small_intervals) > 10:
                print(f"  ... and {len(small_intervals)-10} more")
    
    # Final verdict
    print(f"\n{'='*70}")
    print(f"Final Verdict")
    print(f"{'='*70}\n")
    
    if points_before_imu == 0 and points_after_imu == 0:
        print("✓ ALIGNED: All point cloud timestamps are within IMU timestamp range")
    else:
        print("✗ NOT FULLY ALIGNED: Some point cloud timestamps are outside IMU range")
        if misaligned_points:
            print(f"\nFirst few misaligned points:")
            for idx, ts, reason in misaligned_points[:5]:
                print(f"  Point {idx}: {ts} ns - {reason}")
    
    # Check if data is temporally aligned
    if abs(time_offset_start) < 0.1 and abs(time_offset_end) < 0.1:
        print("✓ Temporal alignment is good (offsets < 0.1s)")
    else:
        print("⚠ Temporal alignment may need adjustment")
    
    print(f"\n{'='*70}")
    print(f"Interval Distribution Summary")
    print(f"{'='*70}\n")
    
    if len(point_timestamps) > 1:
        # Use message timestamp (hardware time), not receive timestamp
        time_diffs_sorted = sorted([(point_timestamps[i][1] - point_timestamps[i-1][1]) / 1e9 
                                     for i in range(1, len(point_timestamps))])
        num_diffs = len(time_diffs_sorted)
        
        print(f"Total intervals: {num_diffs}")
        print(f"Intervals < average ({avg_diff:.6f}s): {len([d for d in time_diffs_sorted if d < avg_diff])} ({len([d for d in time_diffs_sorted if d < avg_diff])/num_diffs*100:.2f}%)")
        print(f"Intervals ≈ average (±10%): {len([d for d in time_diffs_sorted if abs(d - avg_diff) <= avg_diff * 0.1])} ({len([d for d in time_diffs_sorted if abs(d - avg_diff) <= avg_diff * 0.1])/num_diffs*100:.2f}%)")
        print(f"Intervals > average: {len([d for d in time_diffs_sorted if d > avg_diff])} ({len([d for d in time_diffs_sorted if d > avg_diff])/num_diffs*100:.2f}%)")
        print(f"Intervals > 2x average: {len([d for d in time_diffs_sorted if d > 2 * avg_diff])} ({len([d for d in time_diffs_sorted if d > 2 * avg_diff])/num_diffs*100:.2f}%)")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_timestamp_alignment.py <path_to_bag_directory>")
        print("Example: python3 check_timestamp_alignment.py livox_bag_1228")
        print("Note: Provide the bag directory path, not the .db3 file")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    
    # If user provided .db3 file, extract directory
    if bag_path.endswith('.db3'):
        bag_path = str(Path(bag_path).parent)
    
    print(f"Reading ROS2 bag: {bag_path}")
    
    # Extract timestamps
    print("Extracting IMU timestamps...")
    imu_timestamps = get_timestamps_from_bag(bag_path, "/livox/imu")
    
    print("Extracting Point Cloud timestamps...")
    point_timestamps = get_timestamps_from_bag(bag_path, "/livox/lidar")
    
    # Check alignment
    check_alignment(imu_timestamps, point_timestamps)


if __name__ == "__main__":
    main()

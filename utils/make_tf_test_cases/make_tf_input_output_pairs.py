import argparse
import random
from pathlib import Path

import numpy as np
import rosbag
import rospy
import tf2_ros
import yaml

RNG = random.Random(20111111)


def populate_tf_tree(buffer: tf2_ros.Buffer):
    def _create_random_tf(
        frame_id: str,
        child_frame_id: str,
        stamp: int,
    ):
        transform = tf2_ros.TransformStamped()
        transform.header.frame_id = frame_id
        transform.header.stamp = rospy.Time.from_seconds(stamp / 1e9)
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = RNG.random()
        transform.transform.translation.y = RNG.random()
        transform.transform.translation.z = RNG.random()
        quat = np.array([RNG.random() * 2 - 1 for _ in range(4)])
        quat /= np.linalg.norm(quat)
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])
        return transform, {
            "is_static": False,
            "timestamp": transform.header.stamp.to_nsec(),
            "frame_id": transform.header.frame_id,
            "child_frame_id": transform.child_frame_id,
            "translation": [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            "rotation": [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ],
        }

    transforms = []

    # First set is fixed, so that loop can reference something
    children_to_parent_mapping = {
        "odom": "world",
    }
    cur_time_ns = int(1321038671 * 1e9)
    tf_msg, tf_yaml = _create_random_tf("world", "odom", cur_time_ns)
    buffer.set_transform(tf_msg, "")
    transforms.append(tf_yaml)
    min_times = {"odom": rospy.Time.from_sec(cur_time_ns / 1e9)}
    max_times = {"odom": rospy.Time.from_sec(cur_time_ns / 1e9)}

    for idx in range(20):
        child_frame_id = f"frame_{idx}"
        frame_id = RNG.choice(list(children_to_parent_mapping.keys()))
        tf_msg, tf_yaml = _create_random_tf(frame_id, child_frame_id, cur_time_ns)
        buffer.set_transform(tf_msg, "")
        transforms.append(tf_yaml)
        children_to_parent_mapping[child_frame_id] = frame_id
        min_times[child_frame_id] = rospy.Time.from_sec(cur_time_ns / 1e9)
        max_times[child_frame_id] = rospy.Time.from_sec(cur_time_ns / 1e9)

    # Generate transforms
    keys = list(children_to_parent_mapping.keys())
    for idx in range(1000):
        child_frame_id = RNG.choice(keys)
        frame_id = children_to_parent_mapping[child_frame_id]

        tf_msg, tf_yaml = _create_random_tf(frame_id, child_frame_id, cur_time_ns)
        buffer.set_transform(tf_msg, "")
        transforms.append(tf_yaml)
        max_times[child_frame_id] = rospy.Time.from_sec(cur_time_ns / 1e9)

        # Allow at most one second gap
        cur_time_ns += RNG.randint(0, 1_000_000_000)

    return transforms, min_times, max_times, set(children_to_parent_mapping.keys()) | {"world"}


def populate_tf_tree_from_bag(buffer: tf2_ros.Buffer, bag_path: Path):
    bag = rosbag.Bag(str(bag_path))
    transforms = []

    min_times = {}
    max_times = {}
    all_frames = set()
    for topic, msg, ts in bag.read_messages(["/tf", "/tf_static"]):
        is_static = topic != "/tf"

        for transform in msg.transforms:
            if is_static:
                buffer.set_transform(transform, "")
            else:
                buffer.set_transform_static(transform, "")

            # Create a yaml file that can be read by tf2_ros tests
            transforms.append({
                "is_static": is_static,
                "timestamp": transform.header.stamp.to_nsec(),
                "frame_id": transform.header.frame_id,
                "child_frame_id": transform.child_frame_id,
                "translation": [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                "rotation": [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ],
            })

            # For each transform keep track of the earliest and latest times
            if transform.child_frame_id not in min_times:
                min_times[transform.child_frame_id] = transform.header.stamp

            max_times[transform.child_frame_id] = transform.header.stamp
            all_frames.add(transform.child_frame_id)
            all_frames.add(transform.header.frame_id)
    return transforms, min_times, max_times, all_frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-file", type=Path, help="If realistic is specified, this path is used", default=Path(__file__).parent / "tfs.bag")
    parser.add_argument("--realistic", action="store_true", help="If set, the bag file is used to generate transforms. Otherwise, random tree is generated")
    args = parser.parse_args()

    # Generate TF Tree and other artifacts
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration.from_sec(1e9))
    if args.realistic:
        transforms, min_times, max_times, all_frames = populate_tf_tree_from_bag(buffer, args.bag_file)
    else:
        transforms, min_times, max_times, all_frames = populate_tf_tree(buffer)

    with open("transforms.yaml", "w") as f:
        yaml.safe_dump_all(transforms, f)

    # Next generate example inputs
    inputs = []
    outputs = []
    num_examples = 0
    num_rotation_samples = 0
    frame_strings = list(all_frames)
    min_ts = max(min_times.values())
    max_ts = max(max_times.values())
    while num_examples < 100:
        frame_id, child_frame_id = RNG.choices(frame_strings, k=2)

        # We want to have complex examples
        if frame_id == child_frame_id:
            continue

        min_ts = max(min_times.get(frame_id, min_ts), min_times.get(child_frame_id, min_ts))
        max_ts = min(max_times.get(frame_id, max_ts), max_times.get(child_frame_id, max_ts))

        # with valid times
        if max_ts < min_ts:
            continue

        time = RNG.uniform(min_ts, max_ts)

        try:
            transform = buffer.lookup_transform(frame_id, child_frame_id, time)
        except:
            continue

        inputs.append({
            "time": time.to_nsec(),
            "frame_id": frame_id,
            "child_frame_id": child_frame_id
        })
        outputs.append({
            "timestamp": transform.header.stamp.to_nsec(),
            "frame_id": transform.header.frame_id,
            "child_frame_id": transform.child_frame_id,
            "translation": [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            "rotation": [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ],
        })

        num_examples += 1

    # Save inputs and outputs
    with open("inputs.yaml", mode="w") as f:
        yaml.safe_dump_all(inputs, f)
    with open("outputs.yaml", mode="w") as f:
        yaml.safe_dump_all(outputs, f)


if __name__ == '__main__':
    main()

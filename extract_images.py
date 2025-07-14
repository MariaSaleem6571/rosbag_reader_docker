#!/usr/bin/env python3
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import csv
from datetime import datetime

def format_timestamp(ts_ns):
    secs = ts_ns // 10**9
    nsecs = ts_ns % 10**9
    dt_str = datetime.utcfromtimestamp(secs).strftime('%Y%m%dT%H%M%S')
    return f"{dt_str}_{nsecs:09d}"

def extract_paired_images(bag_path, output_dir, topic1, topic2, time_tolerance_ns=50000000):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bridge = CvBridge()
    bag = rosbag.Bag(bag_path, "r")

    images_topic1 = {}
    images_topic2 = {}

    print("Reading messages from bag...")
    for topic, msg, t in bag.read_messages(topics=[topic1, topic2]):
        timestamp_ns = msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if topic == topic1:
            images_topic1[timestamp_ns] = cv_img
        elif topic == topic2:
            images_topic2[timestamp_ns] = cv_img

    bag.close()
    print(f"Read {len(images_topic1)} images from {topic1}")
    print(f"Read {len(images_topic2)} images from {topic2}")

    ts1 = sorted(images_topic1.keys())
    ts2 = sorted(images_topic2.keys())

    pairs = []
    i, j = 0, 0
    while i < len(ts1) and j < len(ts2):
        diff = ts1[i] - ts2[j]
        if abs(diff) <= time_tolerance_ns:
            pairs.append((ts1[i], ts2[j]))
            i += 1
            j += 1
        elif diff < 0:
            i += 1
        else:
            j += 1

    print(f"Found {len(pairs)} matching pairs with tolerance {time_tolerance_ns / 1e6} ms")

    csv_path = os.path.join(output_dir, "paired_image_timestamps.csv")
    with open(csv_path, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["pair_id", "cartesian_filename", "polar_filename", "secs", "nsecs", "iso_timestamp"])

        for idx, (ts_cart, ts_polar) in enumerate(pairs, start=1):
            pair_id = f"{idx:06d}"  # zero-padded sequence number, e.g. 000001

            secs = ts_cart // 10**9
            nsecs = ts_cart % 10**9
            iso_timestamp = datetime.utcfromtimestamp(secs).strftime('%Y-%m-%dT%H:%M:%S') + f".{nsecs:09d}Z"

            cart_filename = f"{pair_id}_cartesian.png"
            polar_filename = f"{pair_id}_polar.png"

            cart_filepath = os.path.join(output_dir, cart_filename)
            polar_filepath = os.path.join(output_dir, polar_filename)

            cv2.imwrite(cart_filepath, images_topic1[ts_cart])
            cv2.imwrite(polar_filepath, images_topic2[ts_polar])

            csv_writer.writerow([pair_id, cart_filename, polar_filename, secs, nsecs, iso_timestamp])

            if idx % 100 == 0:
                print(f"{idx} pairs saved...")

    print(f"Extraction complete. {len(pairs)} image pairs saved to {output_dir}")
    print(f"Timestamps saved to {csv_path}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 5:
        print("Usage: python3 extract_images.py <bagfile.bag> <output_folder> <topic1> <topic2>")
        print("Example: python3 extract_images.py data.bag output /soundmetrics_aris3000/cartesian /soundmetrics_aris3000/polar")
        sys.exit(1)

    bagfile = sys.argv[1]
    output_folder = sys.argv[2]
    topic1 = sys.argv[3]
    topic2 = sys.argv[4]

    extract_paired_images(bagfile, output_folder, topic1, topic2)

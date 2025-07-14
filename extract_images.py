#!/usr/bin/env python3
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import csv
from datetime import datetime

def extract_images(bag_path, output_dir, topic_name):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bridge = CvBridge()
    bag = rosbag.Bag(bag_path, "r")

    csv_path = os.path.join(output_dir, "image_timestamps.csv")
    with open(csv_path, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Write header row
        csv_writer.writerow(["filename", "secs", "nsecs", "iso_timestamp"])

        count = 0
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # Convert ROS Image message to OpenCV image
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Create filename with timestamp (seconds.nanoseconds)
            timestamp = msg.header.stamp
            filename = f"image_{timestamp.secs}_{timestamp.nsecs}.png"
            filepath = os.path.join(output_dir, filename)

            # Save image
            cv2.imwrite(filepath, cv_img)

            # Convert to ISO timestamp string for readability
            iso_timestamp = datetime.utcfromtimestamp(timestamp.secs).strftime('%Y-%m-%dT%H:%M:%S') + f".{timestamp.nsecs:09d}Z"

            # Write CSV row
            csv_writer.writerow([filename, timestamp.secs, timestamp.nsecs, iso_timestamp])

            count += 1
            if count % 100 == 0:
                print(f"{count} images saved...")

    bag.close()
    print(f"Extraction complete. {count} images saved to {output_dir}")
    print(f"Timestamps saved to {csv_path}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 4:
        print("Usage: python3 extract_images.py <bagfile.bag> <output_folder> <topic_name>")
        sys.exit(1)

    bagfile = sys.argv[1]
    output_folder = sys.argv[2]
    topic = sys.argv[3]

    extract_images(bagfile, output_folder, topic)

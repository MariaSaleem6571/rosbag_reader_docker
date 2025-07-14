FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy

WORKDIR /data

COPY extract_images.py /data/extract_images.py

CMD ["bash"]


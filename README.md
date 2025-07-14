# Rosbag Reader Docker

This project reads ROS bag files and extracts images using Docker.

## How to build and run

Build the Docker container:
```bash
docker-compose build
```

Run the image extraction script (replace `<bagfile>`, `<output_folder>`, and `<topic_name>` with your own values):

```bash
docker compose run rosbag_reader python3 extract_images.py <bagfile> <output_folder> <topic_name>
```
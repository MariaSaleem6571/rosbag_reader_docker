# Rosbag Reader Docker

This project reads ROS bag files and extracts **paired images** from two specified topics using Docker. It saves image pairs with matching timestamps (within a tolerance) and creates a CSV file listing the paired filenames and their timestamps.

## How to build and run

Build the Docker container:

Navigate to project repository

```bash
cd your-project-directory
```

```bash
docker-compose build
```

Run the image extraction script (replace <bagfile>, <output_folder>, <topic1>, and <topic2> with the correct names):

```bash
docker compose run rosbag_reader python3 extract_images.py <bagfile> <output_folder> <topic1> <topic2>
```

## Output

Image pairs are saved in the specified output folder with filenames in the format:

- {pair_id}_cartesian.png
- {pair_id}_polar.png

Each `pair_id` is a zero-padded sequence number (e.g., 000001, 000002, ...).

A CSV file named `paired_image_timestamps.csv` is created in the output folder, containing:

- Paired image filenames  
- Corresponding seconds and nanoseconds timestamps

This allows easy correlation of cartesian and polar images captured at nearly the same time.

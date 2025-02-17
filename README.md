# YOLOv8 ROS Node

This repository contains a ROS node that utilizes [YOLOv8](https://github.com/ultralytics/ultralytics) for real-time object detection using a webcam. The detections are published as ROS messages, and the processed video stream is also published for further use.

## Features
- Uses **YOLOv8** for real-time object detection
- Captures video from a webcam and performs inference
- Publishes detection results and annotated images over ROS topics
- Runs on **GPU** for optimized performance

## Dependencies
Ensure you have the following dependencies installed:

- ROS (Robot Operating System) – Tested with **ROS Noetic**
- Python 3
- OpenCV (`cv2`)
- `sensor_msgs`, `std_msgs`, `cv_bridge` (ROS packages)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- PyTorch (with CUDA support for GPU acceleration)

### Install Dependencies
```bash
# Install ROS dependencies
sudo apt-get install ros-noetic-vision-opencv ros-noetic-cv-bridge

# Install Python dependencies
pip install ultralytics torch torchvision torchaudio opencv-python numpy rospy
```

## Usage
### 1. Clone the Repository
```bash
git clone https://github.com/poommin2543/yolov8_ros.git
cd yolov8_ros
```

### 2. Run the YOLOv8 Node
```bash
python yolotoros.py
```

### 3. ROS Topics Published
- **`/camera/image_raw`** – Publishes the annotated image with bounding boxes
- **`/yolov8/detections`** – Publishes detection results in JSON format

## Node Details
The `YOLOv8Node` class initializes the ROS node, captures video frames, runs inference using YOLOv8, and publishes results. The detections include bounding box coordinates, confidence scores, and class IDs.

### Example Output (Detection JSON Format)
```json
[
  {"x1": 100, "y1": 50, "x2": 200, "y2": 150, "confidence": 0.85, "class_id": 0},
  {"x1": 300, "y1": 100, "x2": 400, "y2": 200, "confidence": 0.92, "class_id": 1}
]
```

## 4. Running the Rover Control Node
The **RoverControlNode** subscribes to the YOLOv8 detection results and controls the movement of a rover based on the detected target.

### **How it Works**
- The node subscribes to the **`/yolov8/detections`** topic to receive object detection data.
- It processes the bounding box coordinates and determines whether the rover should move forward, backward, left, or right.
- The movement commands are published to the **`/cmd_vel`** topic, which controls the rover’s motion.

### **Run the Rover Control Node**
```bash
python cmdvel.py
```

### **ROS Topics Used**
- **Subscribed:** `/yolov8/detections` – Receives detection data from YOLOv8
- **Published:** `/cmd_vel` – Sends velocity commands to the rover

## Stopping the Node
Press `Ctrl + C` in the terminal or press `q` in the OpenCV display window to exit.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing
Feel free to open issues or submit pull requests to improve the functionality of this package.

## Author
Poommin2543 - [GitHub Profile](https://github.com/poommin2543)


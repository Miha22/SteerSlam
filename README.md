## Angular velocity tracking system using Simultaneous Localization and Mapping system for identifying planar movement in 3D space
This is a bachelor degree project based on ORB_SLAM2 algorithm that creates a system for reading and writing image sequences, initializing a Simultaneous Localization and Mapping system, and tracking angular rotation and velocity from video frames. The system is designed for use with first-person driving videos and outputs angular velocity data rendered with steering wheel picture for demonstarting self driving.

### Features

#### Image Sequence Reading:
- Handles loading images from files and extracting timestamp and frame metadata. 
- Reads frames directly from video files and preprocesses them (flipping). 
- Supports .mp4 video formats using ffmpeg lib.

#### Image Sequence Writing:
- Encodes processed frames into a video format (H.264). 
- Manages frame-by-frame transformations and writes the output efficiently.

#### SLAM Initialization and Tracking:
- Initializes a SLAM system using ORB_SLAM2 engine.
- Tracks angular rotation and velocity of objects from video frames.
- Outputs trajectory and angular velocity as json files for further analysis.

### Setup
This module integrates ORB-SLAM2 for trajectory and angular velocity estimation:

```
ORB_SLAM2::System slam("vocabulary.txt", "camera_config.yml", ORB_SLAM2::System::MONOCULAR, true);
slampilot::TrackImageSequence(&slam, *image_source, "trajectories.json", &sink);
```

#### TrackImageSequence:
Processes video frames using the SLAM system, extracting pose, rotation and translation data then generates trajectories as a long .json file.

#### Trajectory Analysis:
1. Computes PCA *(Principal Component Analysis)* to determine dominant motion (for driving it is mostly the X and Y planes, since driving hills and mountains are rare).
2. Projects directions onto a horizontal plane.
3. Computes angular velocity based on consecutive frames.
4. Component **rendering** is responsible for generating final video file with dynamic steering wheel that rotates according to planar movement.


### Dependencies
- OpenCV (2.4.3)
- FFmpeg
- ORB-SLAM2 (modified)
- Eigen3 vectors lib
- nlohmann/json

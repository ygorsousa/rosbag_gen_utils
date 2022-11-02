# rosbag_gen_utils
A ROS package for generating rosbags from sequences of image files.

In the current version, the package provides two nodes for rosbag generation:
- `bag_from_images_hz`, which allows the generation from a sequence of image files with the given encoding type and frames per second;
- `bag_from_bgr_depth_times`, which allows the generation from bgr images and depth images. The node uses the times typically contained in the image file names to calculate the frame rates. This solution was adapted to convert the sequences of image files from the [SUN3D](https://sun3d.cs.princeton.edu/) dataset, which provides times in the "id-time" pattern (eg.: 0000210-000007071876). You should customize the solution to suit your needs.

Tested on Ubuntu 18.04 with ROS Melodic and Python 2.7, but it should work on other versions.

## Installation
Clone the repository in the `src` folder of your catkin workspace:

```
git clone https://github.com/ygorsousa/rosbag_gen_utils.git
```

Then go to the root folder of your catkin workspace and run:

```
catkin build
source devel/setup.bash (if needed)
```

## Usage 

To generate a bag using `bag_from_images_hz`, run (pay attention to the parameters):
```
rosrun rosbag_gen_utils bag_from_images_hz.py _images_path:=PATH _topic:=TOPIC _frequency:=FPS _encoding:=ENC _bag_name:=NAME _out_path:=PATH_OUT
```
- `PATH`: path to the sequence of image files; 
- `TOPIC`: topic in which the images data will be written in the rosbag (eg.: /camera/rgb/image_raw); 
- `FPS`: number of frames (images) per second; 
- `ENC`: the encoding of the image data (eg.: bgr8); 
- `NAME`: name of the bag file (eg.: generated-rgb);
- `PATH_OUT`(optional, default: package root folder): path to save the `.bag` file.

In a similar way, to generate a bag using the `bag_from_bgr_depth_times` node, run (note the new parameters):
```
rosrun rosbag_gen_utils bag_from_bgr_depth_times.py _bgr_images_path:=PATH_BGR _depth_images_path:=PATH_DEPTH _bgr_topic:=TOPIC_BGR _depth_topic:=TOPIC_DEPTH _bag_name:=NAME _multiplier:=MULTI _out_path:=PATH_OUT
```
- `PATH_BGR`: path to the sequence of bgr image files; 
- `PATH_DEPTH`: path to the sequence of depth image files; 
- `TOPIC_BGR`: topic in which the bgr images data will be written in the rosbag (eg.: /camera/rgb/image_raw); 
- `TOPIC_DEPTH`: topic in which the depth images data will be written in the rosbag (eg.: /camera/depth/image_raw); 
- `NAME`: name of the bag file (eg.: generated-bgr-depth);
- `MULTI`(optional, default: 1.0): a number that will decrease the frequency (FPS) of the data in the rosbag by multiplying the times obtained from the image file names; 
- `PATH_OUT`(optional, default: package root folder): path to save the `.bag` file.

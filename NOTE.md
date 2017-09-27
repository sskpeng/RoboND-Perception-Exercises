# RoboND-Perception-Exercises Note
This is a note taken for Module 4 Perception in RoboND. This module consisted the following lessons:
1. Perception overview
2. Introduction to 3D Perception
3. Calibration, Filtering, and Segmentation
4. Clustering for Segmentation
5. object Recognition

This document is meant for noting the important steps taught during each sections.

## Introduction to 3D Perception
### Creating a point could from RGB-D data
```
# Get point color for pixel
pixel_color = (image[y,x,0],image[y,x,1],image[y,x,2])

# Get point depth for each particle
depth = depth_map[y,x]
```
## Calibration, Filtering, and Segmentation
### Calibration Pattern
```
$ cd ~/RoboND-Perception-Exercises/python-pcl
$ python setup.py build
$ sudo python setup.py install
```

### Install pcl-tools
```
$ sudo apt-get install pcl-tools
```



## Documentation for `pcl_helper.py`

`pcl_helper.py` contains useful functions for working with point cloud data with ROS and PCL.  The file itself is located in `Exercise-2/sensor_stick/scripts/`.  While the helper functions are required for Exercise-2, they could also come in handy if you want to explore more deeply in Exercise-1.  Here's a brief description of the contents:

#### Functions:
`random_color_gen()`
```
Generates a random set of r,g,b values
Return: a 3-tuple with r,g,b values (range 0-255)
```

`ros_to_pcl(sensor_msgs/PointCloud2)`
```
Converts sensor_msgs/PointCloud2 to XYZRGB Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`pcl_to_ros(pcl.PointCloud_PointXYZRGB)`
```
Converts XYZRGB Point Cloud to sensor_msgs/PointCloud2
Return: sensor_msgs/PointCloud2
```

`XYZRGB_to_XYZ(XYZRGB_cloud)`
```
Converts XYZRGB Point Cloud to XYZ Point CLoud
Return: pcl.PointCloud
```

`XYZ_to_XYZRGB(XYZ_cloud, color)`
```
Takes a 3-tuple as color and adds it to XYZ Point Cloud
Return: pcl.PointCloud_PointXYZRGB
```

`rgb_to_float(color)`
```
Converts 3-tuple color to a single float32
Return: rgb packed as a single float32
```

`get_color_list(cluster_count)`
```
Creates a list of 3-tuple (rgb) with length of the list = cluster_count
Return: get_color_list.color_list
```

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
Use OpenCV functions `findCHessboardCorners()` and `drawChessboardCorners()` to automatically find and draw the "inner corners" on your images of the chessboard pattern.


```
# Find the chessboard corners
ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

# If found, draw corners
if ret == True:
    # Draw and display the corners
    cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
    plt.imshow(img)
```

### Filtering
Introducted a several commonly used filters form the Point Cloud Library. There filters are:

1. VoxelGrid Downsamplying Filter
2. ExtrachIndices Filter
3. PassThrough Filter
4. RANASAC Plane Fitting
5. Outlier Removal Filter

#### Voxel Grid Downsampling

Voxel is short for "volume element". Similar to "pixel" for 2D pictures. Note: LEAF size is in unit of cubic meter in volume. 

```
# Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()

# Choose a voxel (also known as leaf) size
# Note: this (1) is a poor choice of leaf size   
# Experiment and find the appropriate size!
LEAF_SIZE = 1   

# Set the voxel (or leaf) size  
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

# Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)
```

#### PassThrough Filter
To filter out unecessary information if the target location is known.

```
# PassThrough filter
# Create a PassThrough filter object.
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0
axis_max = 2
passthrough.set_filter_limits(axis_min, axis_max)

# Finally use the filter function to obtain the resultant point cloud. 
cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)
```

####  Random Sample Consensus or "RANSAC"
RANSAC is an algorithm, that you can use to identify points in your dataset that belong to a particular model.

```
# Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance 
# for segmenting the table
max_distance = 1
seg.set_distance_threshold(max_distance)

# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()
```

#### Outlier Removal Filter
To remove noise due to external factors.

```
# Much like the previous filters, we start by creating a filter object: 
outlier_filter = cloud_filtered.make_statistical_outlier_filter()

# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(50)

# Set threshold scale factor
x = 1.0

# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(x)

# Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()
```

## Clustering for Segmentation
Clustering is a process to find similarities among individual points.
K-means Clustering divides points into K clusters based on one or more features. 
and Euclidean CLustering, points closed to each other are clustered together.

Density-Based Spatial Clustering of Applications with Noise (DBSACN). Also known as "Euclidean Clustering". This algorithm is a nice alternative to k-means when you don' t know how many clusters to expect in your data, but you do know something about how the points should be clustered in terms of density (distance between points in a cluster).

#### Euclidean Clustering
In order to perform Euclidean Clustering, you must first construct a k-d tree from the cloud_objects point cloud. The k-d tree data structure is used in the Euclidian Clustering algorithm to decrease the computational burden of searching for neighboring points. 


```
# Euclidean Clustering
white_cloud = # Apply function to convert XYZRGB to XYZ
tree = white_cloud.make_kdtree()
```
```
# Create a cluster extraction object
ec = white_cloud.make_EuclideanClusterExtraction()
# Set tolerances for distance threshold 
# as well as minimum and maximum cluster size (in points)
# NOTE: These are poor choices of clustering parameters
# Your task is to experiment and find values that work for segmenting objects.
ec.set_ClusterTolerance(0.001)
ec.set_MinClusterSize(10)
ec.set_MaxClusterSize(250)
# Search the k-d tree for clusters
ec.set_SearchMethod(tree)
# Extract indices for each of the discovered clusters
cluster_indices = ec.Extract()
```

## Object Recognition

1. Color space
2. HSV space
3. Color Histograms
4. Surface Normals
5. Support Vector Machine (SVM)


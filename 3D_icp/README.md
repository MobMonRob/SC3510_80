**Using ICP to align PointClouds:**

- Functionality:
The algorithm utilizes the Point Cloud Library to align two input PointClouds using ICP principles. The variants for the used functions are chosen with respect to the application of this project which is scanning a surface that is going to be sanded.
The functions chosen are used for:

1) Loading the “.pcd” files and removing any NaN points.
2) Initial aligning both clouds. Including shifting, cropping, and position. (This is done in order to have the overlapping areas of both scans on top of each other)
3) Finding the points normals and downsampling clouds using normal-space-sampling. (This method is chosen since it gives higher points concentration near edges and engravings, which is helpful when dealing with mostly flat surfaces)
4) Finding the downsampled normals and the correspondences between the two PointClouds. (This is important since the clouds have different numbers of points)
5) Estimate the transfer function that aligns the clouds using the Singular-Value-Decomposition matrix factorization method. 
Note: Steps 3-5 are applied to the overlapped area to minimize computational effort and process time.
6) Apply the transfer function on the original source cloud, and view it along with the target cloud to confirm the results.

- Running Instructions:
After downloading the 3D_icp directory, follow these steps:
1) Run the cmake command with the source path set to where the cmake file is, and the build path set to out/build.
2) Add the 2 “.pcd” files to be aligned to the out/build directory, or use the example files provided.
3) Call the executable using Terminal with specifying the names of the 2 “.pcd” files to be aligned. (Example: ./diy_icp pack1.pcd pack2.pcd).

**Functions Definition:**

__**loadFile:**__
Loads a .pcd file into a point cloud and prints out a feedback string on whether the file was loaded successfully or not.
The function uses the loadPolygonFile and then stores the data from the mesh into the cloud object. Lastly, it removes any NaN points from the cloud.

__**initAlign:**__
This function applies the required initial alignment by ICP and crops the point clouds into two chunks in order to fasten processing by ‘focusing’ on the overlapped regions of the clouds.
The source cloud is shifted by 1 on the z-axis in order to have a clear initial distance to work with.
Both clouds are then cropped on the x-axis using PassThrough to result in only the overlapped regions (overlap-source, -target).
The source clouds (original & overlap) are lastly shifted by the value of the sensor displacement.
In order to shift the clouds accurately, it is planned to get the sensor displacement value later through other means, for example, through trackers or through robot arm odometry information.

_**normalSpaceSample**_:
Downsampling the point clouds (i.e overlapped regions) is a crucial step to achieving optimal processing effort and time. Normal-Space-Sampling is a method that creates a _number of samples_ by randomly picking a point from _bins_ that contain multiple points.  

- _setSample_: specifies the number of indices to be sampled.
- _setSeed_: specifies the seed to be used by the random function that picks the points from the bins.
- _setBins_: (x, y, z) set the number of bins (buckets) to be considered when picking the corresponding indices.

**MATCHERS:**
_**findNormalCorrespondences**_:
Finding correspondences is the process of matching each point in a cloud with its respective pair in the other cloud. It is the most consequential process on the results of the whole registration. 
Since the majority of the scanned surface is smooth and straight, the normal-shooting matching method has a high potential in aligning the two clouds successfully and with minimum iterations (i.e only one), at least on the z-direction. 
Here, two parameters need to be specified to carry out the process. Namely, the _k-factor_ and the _maximum distance_.

- _K-factor_: specifies the number of nearest neighbor points to be considered in the target cloud when estimating the target normals, this is set to 8 to match the k-factor of the source normals.
- _Maximum distance_: a threshold value that limits the distance between the point pairs, =10.

_**findIndNormalCorrespondences**_:
In this variant, the function takes in 2 additional parameters for the indices of each cloud to be considered in the process, allowing us to take only certain points into consideration. These could be chosen based on other filtering criteria like normal-angle thresholding or others. 

**REJECTORS:**
_**normalCorrRejector**_:
Using the Surface-Nomral correspondence rejector we can make sure that the angle difference between the normal vectors of the point pairs does not exceed 5 degrees. Therefore, filtering out those relatively false point pairs.  

- _Viewpoint_: is considered to be the centroid of the point cloud.
(edit): shifting the z-component to 100 significantly increased the alignment of the normals, since the z-centroid was BETWEEN the points.

- _KSearch_: the classic normal estimation method, estimates a normal vector at a
point by collecting its k nearest neighbors (k-NN) and fitting a plane on them.
The trade-off here is that smaller k-values are better for estimating the normals inside curves, and higher values are optimal for flat surfaces.
The best experimental value for this application was 8.

- _RadiusSearch_: is the radius of the sphere used as the “search” area. RadiusSearch in this application is not optimal since there is various points-concentration in different regions.

_**one2oneRejector:**_
When multiple points from the source cloud pair with the same point in the target cloud, this function chooses the source point with the least distance to the target point and ignores the remaining correspondences.

_**distanceRejector:**_
this function ignores any correspondece that have a distacne above the specified threshod value.

**FEATURE EXTRACTORS:**
_**findNormals:**_
In order to generate stable normals, we need to identify a _viewpoint_ for the normals to point to.
Additionally, a space-partitioning data structure (_kd-tree object_) is essential to apply the search.
The search area can be identified with one of 2 methods, namely, _KSearch_ and _RadiusSearch_.
They cannot be used at the same time.

_**filterByAngle**_:
This function compares the normal vectors of the points with the surface normal vector of the cloud (currently a vector pointing straight in the z-direction) by calculating the angle between them. Therefore, allowing the identification of edges and curved areas in the point cloud. This comes in handy when aligning the clouds in the x-direction since only the curved areas are of significance. 

//**TOOLS:**
_**findTF**_:
This function uses the Singular-Value Decomposition factorization to return a transfer function that estimates the rigid transformation needed to minimize the distance between the calculated correspondences of the two point-clouds. The distance here is minimized by means of translational and/or rotational motion. 

_**cloudsViewer**_:
This is the visualizing function that is called whenever we want to view the clouds. It also views the surface normal vector specified in the filterByAngle, as well as the correspondences calculated in each step. The source cloud is viewed in green and the target cloud in red.

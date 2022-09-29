**How to build:**
1) Preparation of the point cloud library
The [point cloud library](https://pointclouds.org/) is used but its prebuild ready to use versions do not fit the needs of this project, because they do not contain all needed algorithms. In the pcl documentation there is a tutorial [Fitting trimmed B-splines to unordered point clouds](https://pcl.readthedocs.io/projects/tutorials/en/latest/bspline_fitting.html). In its sub chapter "PCL installation settings" there is explained how to enable the NURMBS and B-splinses modules.

Our code is tested with pcl library version 1.12.1. Newer versions should also work.

So, clone an appropriate version of the point cloud library:

```
git clone https://github.com/PointCloudLibrary/pcl

```


3) Clone the repository and navigate into the main folder of the repository

4) preparation of make

```sh
   mkdir build
   cd build
   cmake-gui
```  

**Using ICP to align PointClouds:**

- Functionality:
The algorithm utilizes the Point Cloud Library to align two input PointClouds using ICP principles. The variants for the used functions are chosen with respect to the application of this project which is scanning a surface that is going to be grinded.
The functions chosen are used for:

1) Loading the “.pcd” files and removing any NaN points and storing them in a point clouds vector.
2) Defining key values from the inputted robot positions.
3) Initial aligning both clouds. Including shifting, cropping, and position. (This is done in order to process only the overlapping areas further)
4) Finding the points normals and downsampling clouds using normal-space-sampling. (This method is chosen since it gives higher points concentration near edges and engravings, which is helpful in eliminating points of flat, unhelpful areas)
5) Initial registration: (sets up the clouds as close as possible for the next detail-registration process)
   5.1) Finding the normal-shooting correspondences between the two PointClouds. (This method works best with surfaces that are mostly flat. The parameters are set for far-distaned correspondenes)
   5.2) Rejecting undesired correspondences based on normal-angle difference and one-to-many behaviour. 
6) Targeting the points that lie within curved areas, i.e grooves (details).
7) Detail registration loop:
   7.1) Finding the normal-shooting correspondences between the two PointClouds. (The parameters are set for close-distaned correspondenes, and operate on details only)
   7.2) Rejecting undesired correspondences based on normal-angle difference and one-to-many behaviour.
   7.3) Iterating the registration until a convergence criterium is met. (translation or rotation)
8) Final surface registration loop:
   8.1) Finding the normal-shooting correspondences between the two PointClouds. (The parameters are set for close-distaned correspondenes, and operate on all points)
   8.2) Rejecting undesired correspondences based on normal-angle difference and one-to-many behaviour.
   8.3) Iterating the registration until a convergence criterium is met. (translation or rotation)
9) Combining the now-aligned target and source clouds into one cloud.
10) Iterating 3-9 using the new merged cloud and the next one the vector until the point clouds vector is finished.
11) Saving the total cloud of all the clouds combined in a pcd file.

**Running Instructions:**
After downloading the 3D_icp directory, follow these steps:
1) Run the cmake command with the source path set to where the cmake file is, and the build path set to out/build. (adjust the path to find the pcl-1.12 in CMakeLists.txt if needed)
2) Add the “.pcd” files to be aligned to the build directory. Some example files are provided in this repository.
3) In the terminal, call the executable followed by the “.pcd” files to be aligned, then the ".csv" positions file and lastly the name of the file ".pcd" to save the resultant registered cloud. 

_Example:_ ./diy_icp linear_scan_1.pcd linear_scan_2.pcd linear_scan_3.pcd linear_scan_positions.csv linear_scan_123.pcd

**Functions and Class Definition:**

_**Positions:**_
This class is used to create objects that hold the position values of the current source and target clouds.

**//TOOLS:**

_**loadFile:**_
Loads a .pcd file into a point cloud and prints out a feedback string on whether the file was loaded successfully or not.
The function uses the loadPolygonFile and then stores the data from the mesh into the cloud object. Lastly, it removes any NaN points from the cloud.

_**preAlign:**_
This function aligns the two clouds based on the inputted position values.

_**cropClouds:**_
This function crops the overlapped regions of the clouds for faster processing.
Both clouds are cropped depending on the specified direction resulting in the new clouds "overlap-source" and "overlap-target".

_**normalSpaceSample**_:
Downsampling the point clouds (i.e overlapped regions) is a crucial step to achieving optimal processing effort and time. Normal-Space-Sampling is a method that creates a _number of samples_ by randomly picking a point from _bins_ that contain multiple points.  

- _setSample_: specifies the number of indices to be sampled.
- _setSeed_: specifies the seed to be used by the random function that picks the points from the bins.
- _setBins_: (x, y, z) set the number of bins (buckets) to be considered when picking the corresponding indices.

_**findTF**_:
This function uses the Singular-Value Decomposition factorization to return a transfer function that estimates the rigid transformation needed to minimize the distance between the calculated correspondences of the two point-clouds. The distance here is minimized by means of translational and/or rotational motion. 

_**cloudsViewer**_:
This is the visualizing function that is called whenever we want to view the clouds. It also views the point normals, as well as the correspondences calculated in each step. The source cloud is viewed in green and the target cloud in red. 
Note: 2 overloads are also available. One to views 2 clouds without correspondences, and another to view 1 cloud only.

_**saveFile**_:
Saves the specified cloud into a file with the specified name in .pcd format.

_**readCSV**_:
Extracts the positions inside the ".csv" file and reutrns them as a vector of vectors of position pairs (axis_name/type, value).

_**setPositions**_:
Saves the individual positions extracted from the scan positions vector into a _**Positions**_ object.
Note: an overlaod function is available that saves indivdual positions extracted from the minimum and maximum values of a point cloud.

_**getDirection**_:
ddecides the direction of scanning based on the extrimum values of the source and target clouds.

_**combineClouds**_:
Combines the input source and target clouds into a new sum cloud. It also takes in the position object of each cloud as well as the direction to take the overlapped areas in consideration.

_**nullCloud**_:
It is used to shift the 3D minimum point of the specified cloud to (0, 0, 0).

**//MATCHERS:**

_**findNormalCorrespondences**_:
Finding correspondences is the process of matching each point in a cloud with its respective pair in the other cloud. It is the most consequential process on the results of the whole registration. 
Since the majority of the scanned surface is smooth and straight, the normal-shooting matching method has a high potential in aligning the two clouds successfully and with minimum iterations (i.e only one), at least on the z-direction. 
Here, two parameters need to be specified to carry out the process. Namely, the _k-factor_ and the _maximum distance_.

- _K-factor_: specifies the number of nearest neighbor points to be considered in the target cloud when estimating the target normals, this is set to 8 to match the k-factor of the source normals.
- _Maximum distance_: a threshold value that limits the distance between the point pairs, =10.

_**findIndNormalCorrespondences**_:
In this variant, the function takes in 2 additional parameters for the indices of each cloud to be considered in the process, allowing us to take only certain points into consideration. These could be chosen based on other filtering criteria like normal-angle thresholding or others. 

**//REJECTORS:**

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

**//FEATURE EXTRACTORS:**

_**findNormals:**_
In order to generate stable normals, we need to identify a _viewpoint_ for the normals to point to.
Additionally, a space-partitioning data structure (_kd-tree object_) is essential to apply the search.
The search area can be identified with one of 2 methods, namely, _KSearch_ and _RadiusSearch_.
They cannot be used at the same time.

_**filterByAngle:**_
This function is used to return an indices list of all points with normals higher/lower than the specified angle with respect to a specified vector.
In this application, it used to return the non-flat points; i.e, points with a normal-angle difference higher than 10 with respect to a straight-vector in z-direction.

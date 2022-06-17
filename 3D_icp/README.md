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
3) Call the executable using Terminal with specifying the names of the 2 “.pcd” files to be aligned. (Example: ./diy_icp example1.pcd example2.pcd).

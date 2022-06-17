**Point Clouds Generation:**

- Program functionality:
pcd_gen.cpp is used to generate “.pcd” point cloud files from the scans taken using the SurfaceControl XXXX. The program mainly calls 6 functions in order to achieve the following:

1) Find the SurfaceControl scanner.
2) Connect to the scanner.
3) Set some scanning parameters
4) Setup the data transfer between the scanner and the PC.
5) Trigger the scanner to take single-shot scans.
6) Process the scanned shots by creating point clouds and generating the “.pcd” files.

- PointClouds properties:
The generated PointClouds are unorganized, meaning their height = 1 and width = number of points. The scanner generates XYZ points without any information about color, intensity, or depth.
The SurfaceControl scanner uses has great accuracy and therefore it creates very dense PointClouds with respect to the small scanned area. 

- Running instructions:
To be able to run the file functionally, please download the 3D_Scanner directory as it is and follow these steps:
1) Setup the environment by calling or copying the commands listed in env.sh into the Terminal.
2) Run the cmake command with the source path set to where the cmake file is, and the build path set to out/build.
3) Call the executable using Terminal with specifying the name of the “.pcd” file to be generated. (Example: ./pcd_gen Example.pcd)
   **Note:** Make sure to use different names for multiple files otherwise the latter one will overwrite the first.

The generated files can be found in the _out/build_ directory. 

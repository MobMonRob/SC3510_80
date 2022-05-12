# SC3510_80
Code to integrate the 3d scanning sensor into the RaHM-Lab software infrastructure.

**Windows Set-up:**
In order to establish a connection with the surfaceControl_3D, the network card needs to be configured as follows:
1) Jumbo Packet = 9014 Byte
2) Interrupt-Drosselung = Aktiviert
3) Interrupt-Drosselungsrate =  Adaptiv
4) Empfangspuffer = 2048

A connection was successfully established with the 3Dinspect software, and the following points are noteworthy:
1) The file format of the generated 3D data is **.me3dpc**.
2) It was possible to create points/planes on the scanned object, and then calculate the distances/angles between them, alongside many other functionalities. 


**Linux Set-up:**
Using the provided SDK and the following configurations, a connection with the SC3D was established. The compilation of the provided SDK was done with the help of CMake tools. The configurations required were:

  1) Specify the path for the library that includes all necessary header/binary files. (Use the -D prefix with cmake command to refer to the _C++/cmake_        directory)
  2) Specify the path for the two variables _LD_LIBRARY_PATH_ and _GENICAM_ROOT_V3_1_. (Use the commands written in env.sh to refer to the _/Libs_              directory)

Like any CMake project, after configuring the project by building _CMakeLists.txt_, and building the project by building _Makefile_, an executable for each C++ source file will be created in the specified build path, _.../out/build_ in this project.

Including a desktop ethernet switch made the source code files more functional and convenient as it solved some connectivity issues.  

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

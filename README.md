# SC3510_80
Code to integrate the 3d scanning sensor into the RaHM-Lab software infrastructure.
<u>Windows Set-up:</u>
In order to establish a connection with the surfaceControl_3D, the network card needs to be configured as follows:
1) Use the largest MTU value possible (Jumbo frames): **sudo ip link set enp3s0 mtu 9000**
2) Enable adaptive throttle interrupting: **sudo ethtool -C enp3s0 rx-usecs 1** _Problem: changes are not applied_
3) Use the largest receive buffer value possible: **ethtool -G enp3s0 rx 4028 tx 4028** _Problem: Cannot get device ring settings: Operation not supported_

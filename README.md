# Introduce
This is an Autoware.Universe repository. 
Time stamp is set in the source code to trace the execution latency.
The launch files are also rewrote for different experiment requirements.
The camera perception function should launch the image remapping node, if not, the Autoware simulation will not execute the tensorrt_yolov node (camera perception).

How to use: Replace the origin src files in Autoware.Universe with this src folder.


## 20240327
I close the camera perception the traffic light function for testing the lidar perception latency.

You can check the launch files and open it again.

I also write some very simple python code for data processing.
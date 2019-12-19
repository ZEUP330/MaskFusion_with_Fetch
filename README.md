# MaskFusion_with_Fetch
  
  modified from [MaskFusion](https://github.com/martinruenz/maskfusion)
  
  get the 3d-information (bounding cube[xyz, lwh])
  try to get once:
  ```shell
    rostopic echo -n 1 /maskfusion/lwh_xyz 
  ```
  
  ![Figure MaskRCNN](https://github.com/ZEUP330/MaskFusion_with_Fetch/blob/master/figures/maskfusion_6coke_can.png)

According to your needs, reading the readme file to download and compile the dependencies, and finally compiling your maskfusion. 

I modified the maincontroller.cpp and Cmakelist.txt in the gui folder to make it support ros topic. (embedded catkin_make into cmake)

but it still have some problems （segmentation fault, image of kinect and fetch has none value in real world, but virtual world）

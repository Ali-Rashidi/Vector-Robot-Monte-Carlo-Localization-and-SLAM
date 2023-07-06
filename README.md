# Vector-Robot-Monte-Carlo-Localization-and-SLAM
Final project of graduate course "Advanced Robotics" at university of Tehran.  

This repository contians implementations of Monte Carlo Localization(MCL) and Augmented Monte Carlo Localization(AMCL) with some modifications and also there is a simple version SLAM.  

Codes are written in order to work on VECTOR-ROBOT on gazebo simulation and real world.  

** Localization:  

  
Multiple version of localization algorithms are available. the result below is for "AMCL+Random"  

  
![Rec 0040 mp4_snapshot_02 16_ 2023 07 04_07 15 24](https://github.com/Ali-Rashidi/Vector-Robot-Monte-Carlo-Localization-and-SLAM/assets/107252860/be695b22-9f49-4c6e-ba94-c36cc2e75f72)  

After localizing the robot, a kiddnapping precegure happens and the algorithym is able to localize the robot again.    

![Rec 0040 mp4_snapshot_06 09_ 2023 07 04_07 24 26](https://github.com/Ali-Rashidi/Vector-Robot-Monte-Carlo-Localization-and-SLAM/assets/107252860/812b3fe3-22c8-4e15-aec6-2cf03e03cdc3)    

The picture below is the graph that shows the percentage of particles within 5 cm range to the actual robot pose at every iteration.   

![image](https://github.com/Ali-Rashidi/Vector-Robot-Monte-Carlo-Localization-and-SLAM/assets/107252860/71e2b94a-7ff8-4d74-853e-da50734900a2)  



    ---------------------------------------------------------------------------------------------------------------------------------------

** SLAM:  

The code written for GridBasedSLAM works decent as the robot has only one simple beam laser and the algorithym is the simplest form of SLAM.  
There is some issue with the automatic movement of the robot because it can't explore the whole world and stucks in a region.   
Any suggestion on better movement strategies will be welcomed!  


  ![Capture2](https://github.com/Ali-Rashidi/Vector-Robot-Monte-Carlo-Localization-and-SLAM/assets/107252860/f2fdaa07-3104-429d-97fa-d4873785dfaa)








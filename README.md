# phantomx_reactor

![image](https://github.com/user-attachments/assets/8e40128d-37ad-4ffe-b645-435b26cb79df)

First, the robot need to be launched in RViz:

    ros2 launch phantomx_reactor kinematics.launch.py here

Second, the robot needs a request in order to get to a goal:

    ros2 run phantomx_reactor kinematics_client.py 0.296 0.148 0.206

Where 0.296 is the desired x, 0.148 the desired y and 0.206 the desired z.

Other suggested goals are:

 - **Goal 1:** 0.323 0.002 0.228
 - **Goal 2:** 0.200 0.125 0.175

If it's necessary to check the kinematics solution, it can be done through the following command:

    ros2 run tf2_ros tf2_echo base_footprint gripper_guide_link

![PhantomX Reactor](https://github.com/user-attachments/assets/c8db91b4-d8cf-4009-9f07-d4a1964a9f9b)

# roboboat_GNC 

## How to ssh to the jetson

1. Connect to the teamInspirationField wifi
2. run: ssh jetson@192.168.8.202 
3. pw: jetsonucsd


## Running the Serial server

1. SSH to the jetson
2. inside the Jetson run: docker run -it --privileged this_actually_works
3. cd ~/ros2_serial_interface
4. . install/setup.bash
5. cd ros2_serial_interface
6. run: python3 serial_server.py

## Running the navigation Node
1. open a new terminal
2. SSH into the Jetson
3. do docker ps and copy the container id of the this_actually_works docker container
4. run: docker exec -it <container-id> /bin/bash
5. cd ~/ros2_serial_interface
6. . install/setup.bash
7. cd ros2_serial_interface
8. vim auto_nav.py
9. paste the code into auto_nav.py
10. run: python3 auto_nav.py 
    
## Running the camera Node
1. open a new terminal
2. ssh into the Jetson
3. run: docker run -it --privileged --network=host camera_node_working_code
4. cd roboboat_code
5. run: python3 buoy_navigation.py

## Moving the boat
1. Move the boat in front of the buoy path using Remote Controller
2. Press the top left button on the Remote Controller to make it autonomous mode
3. Boat should move autonomously through the buoys, completing the **follow the path** task


## Documentation
- [Project Specification](https://drive.google.com/file/d/1p6h2CfVdSFPlanNX6-uoW_igBA2jE8-6/view?usp=sharing)
- [Milestone Report](https://drive.google.com/file/d/1p6h2CfVdSFPlanNX6-uoW_igBA2jE8-6/view?usp=sharing)
- [Final Oral Presentation](https://docs.google.com/presentation/d/15L_Spe2V0VIPEk-d25XDykG1sU-B2HTUzz7Nw4XxNmM/edit?usp=sharing)
- [Technical Report]()

## Contributors
- [Akshara Kuduvalli](https://github.com/akkuduvalli)
- [Ali Alani](https://github.com/a3alani)
- [Dina Dehaini](https://github.com/dinadehaini)
- [Kenzo Ku](https://github.com/kenzoputraku)
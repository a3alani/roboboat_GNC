# roboboat_GNC 

## How to ssh to the jetson

1. Connect to the teamInspirationField wifi
2. run: ssh jetson@192.168.8.202 
3. pw: jetsonucsd


## Running the Serial server

1. SSH to the jetson
2. inside the Jetson run: docker run -it --privileged this_actually_works
3. cd
4. cd ros2_serial_interface
5. . install/setup.bash
6. cd ros2_serial_interface
7. run: python3 serial_server.py

## Running the navigation Node
1. open a new terminal
2. SSH into the Jetson
3. do docker ps and copy the container id of the this_actually_works docker container
4. run: docker exec -it <container-id> /bin/bash
5. cd
6. cd ros2_serial_interface
7. . install/setup.bash
8. cd ros2_serial_interface
9. vim auto_nav.py
10. paste the code into auto_nav.py
11. run: python3 auto_nav.py 
    
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

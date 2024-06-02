# IoT_proj_balloon_humble

/////To Run the program//////

-------Create a workspace----------
Create a workspace in home location

```
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
```

Paste the project folder at location '~/ros2_ws/src/'
Build the workspace

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

-----------Install the dependencies--------------
To install 'dearpygui'
```
pip3 install dearpygui
```


-----------Run the program----------
Go to locaiton of package folder 
'~/ros2_ws/src/IoT-Project-2024/'

```
cd ~/ros2_ws/src/IoT-Project-2024/
ros2 launch project_main simulation_launch.py
```

-----------------Rqt nodes------------------------
Run the following command to view rqt nodes
```
rqt_graph
```

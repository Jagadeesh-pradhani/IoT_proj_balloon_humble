# IoT_proj_balloon_humble


## Create a workspace
Create a workspace in home location

```
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
```

Clone the repo,

```
cd ~/ros2_ws/src
git clone https://github.com/Jagadeesh-pradhani/IoT_proj_balloon_humble.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Install the dependencies
To install 'dearpygui'
```
pip3 install dearpygui
```


## Run the program
Go to locaiton of package folder 
'~/ros2_ws/src/IoT_proj_balloon_humble/IoT-Project-2024/'

```
cd ~/ros2_ws/src/IoT_proj_balloon_humble/IoT-Project-2024/
ros2 launch project_main simulation_launch.py
```




## Rqt nodes
Run the following command to view rqt nodes
```
rqt_graph
```

## Reference
https://fede3751.github.io/IoT-lectures-2024/  <br>
https://github.com/Fede3751?tab=repositories

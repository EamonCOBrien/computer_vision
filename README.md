# computer_vision

## How to run:

1) Launch gazebo: 
`roslaunch neato_gazebo neato_empty_world.launch load_camera:=true`

2) Add models to the gazebo world.

3) Launch rqt window for visualizing from neato camera perspective:
`rosrun rqt_gui rqt_gui`

4) If you haven't before, compile the main script: 
`chmod u+x finite_state.py`

5) Run finite state controller: 
`rosrun computer_vision finite_state.py`
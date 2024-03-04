## Mistakes

- I didn't run the `source catkin_ws/devel/setup.bash` command since following the MuSHR Foxglove docks didn't build it that way

## Setup

- Confirm XQuarts Setup (check below)
- Open VSCode in container
- You make need to run `catkin clean` then `catkin build` in both the mushr_ws and the dependencies_ws folders as the catkin does some stuff to the container
- Navigate to `rangelib_c/pywrapper` and run `sudo python setup.py install` (must be sudo)

## Running muSHR Visualization
- Run `mushr_noetic` in your shell
- Run `source ~/.bashrc` in the container shell
- Run `roslaunch mushr_sim teleop.launch` to start the SIM

## Common commands

`roslaunch cse478 teleop.launch`
`roslaunch cse478 foxgloveteleop.launch`
`rostopic list` Lists all topics currently running (requires roscore/roslaunch to be running)

## X11 XQuartz Setup
For more info check this out: https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088

- Make sure you have XQuarts running
- Make sure you have typed `xhost +localhost` every time you open xquartz
- Run `ps aux | grep Xquartz` on your host machine, if it says "nolisten" anywhere make sure to enable network settings
- Run `export DISPLAY=host.docker.internal:0`
Run:
```
defaults write org.xquartz.X11 enable_iglx -bool YES

# Verify with
defaults read org.xquartz.X11 enable_iglx
# should return: 1
# the restart XQuartz
```

- QT Stuff
    - `export QT_QPA_PLATFORM=offscreen`
    - `pip uninstall opencv-python`
    - `pip install opencv-python-headless`
under XQuartz Preferences (You can get to this via the top mac nav bar with xquarts open) -> Security -> Allow network clients

## Useful Links
https://github.com/prl-mushr/mushr_pf/blob/master/src/mushr_pf/motion_model.py
https://github.com/prl-mushr/mushr_control/blob/master/src/purepursuit.py
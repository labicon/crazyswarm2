# ICON Flying Arena Setup

*Intelligent CONtrol Labratory for the IRL Flying Arena Setup.*

## Instructions

This page should bring you up to speed on how experiments are conducted using
our `crazyflie/crazyswarm` setup. While past experiments used Ubuntu 16.04 on
ROS 1, `crazyswarm2` recently switched to more recent versions of Ubuntu on ROS
2.

1. Clone this repository:
  ```bash
  git clone https://github.com/labicon/crazyswarm2
  ```
1. Go to the [`Crazyswarm2
   Docs`](https://imrclab.github.io/crazyswarm2/installation.html) and follow
   the installation instructions. This lab laptop is currently on Ubuntu Jammy
   (22.04) on ROS2.
1. Ensure the VICON system is powered on and calibrated including the setting of
   the masks.
    - Some of the cameras have undesired effects that need to be accounted for.
      You should be able to see the infrared markers appear in real-time.
    - To validate that VICON is doing its job, do the following:
      ```bash
      # (from this directory)
      cd ../motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture
      # Follow the instructions for the C++ section. I wasn't able to get the python to work.
      cp build/temp.linux-x86_64-3.10/motioncapture_example ~/.local/bin/_motioncapture_example
      cd ~/.local/bin
      ln -s ~/Documents/path/to/crazyswarm2/vicondump ~/.local/bin
      ```
      If `~/.local/bin/` is on your path, which it should be, you should now be
      able to use `vicondump` which has been hardcoded to our system and
      hostname.`
1. To automate the setup, it might be worth looking at some of the aliases I've
   defined in `~/.bash_aliases` to skip some of the repetitive steps.


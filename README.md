- - - - - - - - - - 
install and setup ros
- - - - - - - -- - 

 * sudo apt-get install ros-jade-desktop-full
 * sudo rosdep init
 * rosdep update
 * echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
 * source ~/.bashrc
 * sudo apt-get install python-rosinstall
 * mkdir -p ~/catkin_ws/src
 * cd ~/catkin_ws/src
 * catkin_init_workspace
 * echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

- - - - - - - - - - 
Build
- - - - - - - - - - 
catkin_make --pkg paexample

- - - - - - - -
Requirements
- - - - - - - -
a) fftw: 
sudo apt-get install libfftw3-dev 

b) portaudio:
sudo apt-get install libjack-jackd2-dev
sudo apt-get install portaudio19-dev
sudo apt-get install libportaudiocpp0

c) boost

d) sndfile:
sudo apt-get install libsndfile1-dev 

e) eigen3:
sudo apt-get install libeigen3-dev

f) Based on your Ubuntu version, select the correct ROS distribution. For example, for Ubuntu 14 use (ROS Indigo):
http://wiki.ros.org/indigo/Installation/Ubuntu
For Ubuntu 15 use (ROS Jade):
http://wiki.ros.org/jade/Installation/Ubuntu


- - - - - - - -
Execution instruction
- - - - - - - -
Enter the a package subdirectory to see specific execution instructions

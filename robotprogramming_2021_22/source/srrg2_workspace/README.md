srrg_library_compile instructions:


srrg2 Repo contains: dl.conf , src , README.md


 Prequisits:
 
 ROS should be installed depending on your ubuntu version(ubuntu 20.04 is recomended) (ROS Noetic)
 - Follow instructions in link : http://wiki.ros.org/ROS/Installation
 
 Instll Catkin tools: 
sudo apt-get install pip3
sudo pip3 install -U catkin_tools
 *************************************************************************************************

- Install emacs (sudo apt install emacs)

- Install libraries:
Check
#################################################################################################
- GLUT (sudo apt-get install freeglut3-dev)

- GLFW3 (sudo apt install libglfw3-dev)

- qglviewer (sudo apt-get install libqglviewer-dev-qt5)

- CSParse (sudo apt-get install libsuitesparse-dev)

- Libwebsocket ( sudo apt-get install -y libwebsockets-dev)

- libeigen3-dev (sudo apt install libeigen3-dev)

- ninja-build  (sudo apt install ninja-build )

- libncurses5-dev (sudo apt install libncurses5-dev)

- libreadline-dev (sudo apt install libreadline-dev)
 
- qtdeclarative5-dev (sudo apt install qtdeclarative5-dev)
 
- qt5-qmake (sudo apt install qt5-qmake)
 
- libqglviewer-dev-qt5 (sudo apt install libqglviewer-dev-qt5)
 
- libudev-dev (sudo apt install libudev-dev)

- libgtest-dev (sudo apt install libgtest-dev)
 
- arduino (sudo apt  install arduino)
 
- arduino-mk (sudo apt install arduino-mk)
 
- build-essential (sudo apt install build-essential)
 
- PCL (sudo apt-get install libpcl-dev)

***Check your ROS version : (rosversion)

 ros-<destro>-grid-map-msgs (sudo apt install ros-<destro>-grid-map-msgs)
         ****!!!(Change <destro> with the ROS distributor you are using)!!!****
		    <destro> = 'noetic','melodic', 'kinetic'  etc...
#################################################################################################
**(INSTEAD)**
You can copy this line and paste in terminal to download all packages in one go:

sudo apt-get install freeglut3-dev libglfw3-dev libqglviewer-dev-qt5 libqglviewer-dev-qt5 libsuitesparse-dev libwebsockets-dev libeigen3-dev ninja-build libncurses5-dev libreadline-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 libudev-dev libpcl-dev libgtest-dev arduino arduino-mk build-essential ros-<destro>-grid-map-msgs
#################################################################################################
- Reboot your machine


- go to src, and initialize workspace

(catkin_init_workspace) 

A CMakelist.txt file will be created in src/ folder

 - go back to workspace(srrg2), and compile

 catkin build (takes approx. 15 min)
 
A build/ and devel/ folders will be created in workspace


- source workspace (srrg2)

source devel/setup.bash


- open dl.conf, make sure path until devel folder is correct (Starting from {HOME}/...../srrg2/devel/lib

- go back to srrg2 and source

source devel/setup.bash




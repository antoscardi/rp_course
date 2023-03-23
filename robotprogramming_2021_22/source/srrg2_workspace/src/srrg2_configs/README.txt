This folder contains command line scripts to run various robot tasks
Before running it configure the environment

1. put in your /home/bin/a link to the srrg2_shell binary (<workspace>/devel/lib/srrg2_executor/srrg2_shell) or use the non gui version on a headless system (e.g. rpi on robot)

$> mkdir ${HOME}/bin
$> cd    ${HOME}/bin
$> ln -s <path to the srrg2_shell binary> .

2. edit your /home/dl.conf file by adding the libraries you might need

3. test the program issuing
$> srrg2_shell and seeing if the shell comes up and if it loads all the libs

4. to download the datasets install gdown
   pip install gdown
   
Tools contained:
calibration_2d: run the calibration of platform intrinsics, and extrinsics:
                baseline, kl, kr of a robot and position of the laser scan

slam_2d:        generate 2d maps from a bag

mpr:            mpr stuff

proslam:        generate a 3d graph from a bag with stereo camera

cd ~
git clone https://github.com/sorousharghavan/Udacity_SelfDriving_Car_System_Integration

# conda env
conda env create -f=environment.yml --name carnd-final --debug -v -v

# install dbw_mkz
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash)


# init ros
cd ./Udacity_SelfDriving_Car_System_Integration/ros/src/
rm CMakeLists.txt
catkin_init_workspace
cd ..
catkin_make
#ERROR: Could not find a package configuration file provided by "dbw_mkz_msgs"

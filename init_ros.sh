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

chmod u+x ./src/twist_controller/dbw_node.py ./src/waypoint_loader/waypoint_loader.py ./src/waypoint_updater/waypoint_updater.py ./src/tl_detector/tl_detector.py ./src/tl_detector/light_publisher.py ./src/camera_info_publisher/yaml_to_camera_info_publisher.py

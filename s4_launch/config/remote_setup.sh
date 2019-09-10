export ROS_IP=`hostname -I | cut -d' ' -f1`
export ROS_MASTER_URI=http://`hostname -I | cut -d' ' -f1`:11311
export ROSLAUNCH_SSH_UNKNOWN=1


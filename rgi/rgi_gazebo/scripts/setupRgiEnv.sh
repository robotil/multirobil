#!/bin/sh

##########################
#Initializing environment#
##########################

#making sure the ROS environment is set up
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS environment isn't defined. forced exit"
    exit 1
fi 

#get gazebo prefix
gazebo_prefix=$(pkg-config --variable=prefix gazebo)

#setup file path
setup_file=$gazebo_prefix/share/gazebo/setup.sh
sudo chmod 777 $setup_file

#locate rgi_description package
rgi_dir=$(rospack find rgi_description)

#locate live_bobcat package
#live_rgi_dir=$(rospack find live_bobcat)

############################################
#Adding .gazebo to the simulation resources#
############################################

printf "Adding .gazebo to the simulation resources --- "
if grep -q 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo' $setup_file; then
	printf "already configured\n"
else
	echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo' >> $setup_file
	printf "done\n"
fi


##########################################################
#Generate a model of the RGI6X6 in the gazebo environment#
##########################################################

printf "Copying model meshes to the gazebo folder--- "
#create the model dir if it doesn't exist
if [ ! -d $HOME/.gazebo/models/rgi ]; then
	$(mkdir -p $HOME/.gazebo/models/rgi/meshes)
fi

#copy meshes
$(cp -u $rgi_dir/meshes/* $HOME/.gazebo/models/rgi/meshes)

printf "done\n"

#!/bin/bash

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib:/usr/local/lib64

dataset="Edinburgh-Atrium"

if [ $# -eq 0 ]; then
	echo "Wrong syntax. ./run.sh <agent-id-1> [<agent-id-2> ... <agent-id-n>]"
	
	exit 0
fi

if [ "$dataset" == "PETS-2009" ]; then
	for i in $@
	do
		../bin/DistributedTracker -img ../Datasets/Crowd-PETS09/S2/L1/Time_12-34/View_00$i/frame_0000.jpg -fps 7 -dataset $dataset -agentId $i -loadbg ../backgrounds/S2_L1_00$i.bg &
	done
elif [ "$dataset" == "TUD-Campus" ]; then
	../bin/DistributedTracker -img ../Datasets/TUD-Campus/frame_0000.png -fps 7 -dataset $dataset -agentId 1 &
elif [ "$dataset" == "Edinburgh-Atrium" ]; then
	../bin/DistributedTracker -img ../Datasets/Edinburgh-Atrium-Clips/2014-09-10/Clip-1/image_112950.jpg -fps 5 -dataset ${dataset} -agentId 1 -loadbg ../backgrounds/Edinburgh-Atrium_2014-09-10_Clip-1.bg &
elif [ "$dataset" == "Kinect" ]; then
	# The environment variables are not visible when in root mode.
	sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib:/usr/local/lib64 PTracking_ROOT=${PTracking_ROOT} ../bin/DistributedTracker -kinect -fps 30 -agentId $1
elif [ "$dataset" == "Camera" ]; then
	../bin/DistributedTracker -camera -fps 30 -agentId $1
fi

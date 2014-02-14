#! /bin/sh

echo "Starting rexarm driver monitor script"

while [ `pgrep -f "./maebot_video_teleop"` ]
do
	if [ ! `pgrep -f "./maebot_driver"` ]
	then
		./maebot_driver
	fi
done



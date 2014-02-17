#! /bin/sh

echo "Starting rexarm driver monitor script"

while [ `pgrep -f "./maebot_app"` ]
do
	if [ ! `pgrep -f "./maebot_driver"` ]
	then
		~/ebolson/eecs467/bin/./maebot_driver
	fi
done



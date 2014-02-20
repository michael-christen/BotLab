#! /bin/sh

#trap exiting SIGINT

#exiting() { echo "Received ctrl-c\n"; }

echo "Starting maebot driver"

#while [ `pgrep -f "./maebot_app"` ]
#do
while [ ! `pgrep -f "./maebot_driver"` ]
do
	./maebot_driver
done
#done
echo "Maebot driver started successfully"

while [ `pgrep -f "./maebot_app"` ]
do
#Just hang out til the bot app closes
done

echo "Killing maebot driver"

if [ `pgrep -f "./maebot_driver"`]
then
	kill `pgrep -f "./maebot_driver"`
fi

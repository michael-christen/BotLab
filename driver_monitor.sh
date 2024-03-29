#! /bin/sh

trap exiting SIGINT

EXITED=0
exiting() { EXITED=1; }

echo "Starting maebot driver"

#while [ `pgrep -f "./maebot_app"` ]
#do
while [[ ! `pgrep -f "./maebot_driver"` ]]
do
	./maebot_driver
done
#done
echo "Maebot driver started successfully"

CYCLES=0

while [[ `pgrep -f "./maebot_app"` && $EXITED -eq 0 ]]
do
#Just hang out til the bot app closes
CYCLES=$CYCLES+1
done

echo "Killing maebot driver"

if [[ `pgrep -f "./maebot_driver"` ]]
then
	kill `pgrep -f "./maebot_driver"`
fi

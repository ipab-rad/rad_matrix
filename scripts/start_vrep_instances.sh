#!/bin/bash

while [[ $# > 0 ]]
do
key="$1"

case $key in
    -c|--count)
    COUNT="$2"
    shift # past argument
    ;;
    -h|--hidden)
    HIDDEN=YES
    ;;
    -q|--quit|-k|--kill)
    QUIT=YES
    ;;
    --default)
    echo "DEFAULT OPTION? NOBODY NEEDS THAT!"
    DEFAULT=YES
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done

echo QUIT = "${QUIT}"

if [ "${QUIT}" = "YES" ]; then
   killall -9 vrep
   echo "Killed all vrep instances"
   exit 1
fi

echo COUNT  = "${COUNT}"
echo HIDDEN = "${HIDDEN}"

START=1
END=COUNT
echo "Starting..."
export ROS_MASTER_URI=http://goya:11311

Xvfb :1 -screen 0 1024x768x16 &> xvfb.log  &
DISPLAY=:1
export DISPLAY

for (( c=$START; c<=$END; c++ ))
do
        if [ "${HIDDEN}" = "YES" ]; then
                ./vrep.sh -h &
        else
                ./vrep.sh &
        fi
        sleep 0.5
done

echo
echo "Everything started!"

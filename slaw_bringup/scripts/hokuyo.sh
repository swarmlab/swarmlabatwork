#!/bin/sh
. /opt/ros/fuerte/setup.sh
id=$(rosrun hokuyo_node getID $1 --)

if [ "$id" = "H1009079" ] ; then
    echo "rear"
elif [ "$id" = "H1205008" ] ; then
    echo "front"
fi
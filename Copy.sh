#!/usr/bin/bash
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do
    DIR=$( cd -P "$( dirname "$SOURCE" )" > /dev/null 2>&1 && pwd )
    SOURCE=$(readlink "$SOURCE")
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
DIR=$( cd -P "$( dirname "$SOURCE" )" > /dev/null 2>&1 && pwd )

rm -r /media/$USER/VEX_V5/*

cp "$DIR/Autonomous.py" "$DIR/Constants.py" "$DIR/Odometry.py" "$DIR/Utilities.py" "$DIR/XDrivetrain.py" "/media/$USER/VEX_V5/"

mkdir /media/$USER/VEX_V5/Logs
umount /media/$USER/VEX_V5/

#!/bin/bash

echo "------------Running LCM LOGGER------------"


# LCMLOG_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM"
# cd "$LCMLOG_env"
cd "$1"
lcm-logger "$2"

RUNNING_PID=$!

#Kill the process
cleanup() {
    echo "Killing the MHPC bash..."
    kill $RUNNING_PID
    wait $RUNNING_PID
    exit 0
}

#signals to kill the procs 
trap cleanup SIGTERM SIGINT
#wait for resources release
wait

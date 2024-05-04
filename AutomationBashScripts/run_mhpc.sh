#!/bin/bash


echo "------------Running MHPC------------"

MHPC_env='/media/jnganga/Internal Storage1/jnganga/MPCStack/build/'
cd "$MHPC_env"
./MHPC/mhpc_run &
SIM_PID=$!

#Kill the process
cleanup() {
    echo "Killing the simulation..."
    kill $SIM_PID
    wait $SIM_PID
    exit 0
}

#signals to kill the procs 
trap cleanup SIGTERM SIGINT
#wait for resources release
wait

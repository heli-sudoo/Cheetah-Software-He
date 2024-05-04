#!/bin/bash


echo "------------Running CTRL m s------------"

CTRL_env='/home/jnganga/Desktop/Cheetah-Software-He/build/'
cd "$CTRL_env"
./user/MHPC_LLController/mhpc_llctrl m s &
CTRL_PID=$!

#Kill the process
cleanup() {
    echo "Killing the simulation..."
    kill $CTRL_PID
    wait $CTRL_PID
    exit 0
}

#signals to kill the procs 
trap cleanup SIGTERM SIGINT
#wait for resources release
wait

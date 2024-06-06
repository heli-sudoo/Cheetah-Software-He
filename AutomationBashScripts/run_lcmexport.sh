#!/bin/bash

echo "------------Running LCM EXPORT------------"

# LCMLOG_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM"
LCMTYPES="$(pwd)/../lcm-types"

# cd "$LCMLOG_env"
cd "$1"
lcm-export -m --lcmtypes "$LCMTYPES" "$2"
RUNNING_PID=$!

#mv -- will treat ever the arg as filename. It is needed since arg might contain special characters
#using sed to replace space / / with a backslash
#using sed to replace do \./ with a _ this follows lcm-export format rules
# filename=$(echo "${2}" | sed 's/ /\\\ /g; s/\./_/g')
filename=$(echo "${2}" | sed 's/\./_/g')

mv -- "${filename}.mat" "./MatlabData"


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

#!/bin/bash

echo "------------Running LCM EXPORT------------"

LCMLOG_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM"
LCMTYPES="$(pwd)/../lcm-types"

cd "$LCMLOG_env"
lcm-export -m --lcmtypes "$LCMTYPES" "./$1"
mv "${1}.mat" ./MatlabData
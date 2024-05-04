#!/bin/bash

echo "------------Running LCM LOGGER------------"


LCMLOG_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM"
cd "$LCMLOG_env"
lcm-logger "$1"

#!/usr/bin/env python3

import os
import subprocess


# LCMLOG_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM"
lcmlogexport_path = './../AutomationBashScripts/run_lcmexport.sh'


USE_FLY_OPTIONS = {True,False}

for USE_FLY in USE_FLY_OPTIONS:

    if USE_FLY:
        path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/GRF_Scoped/FLY"
    else:
        path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/GRF_Scoped/NOFLY"

    # Use os.scandir() to get an iterator of directory entries
    with os.scandir(path_fly_no_fly) as entries:
        for file_name in entries:
            if file_name.is_file():
                print(f"---------Processing LCM File: {file_name.path}")

                # LETS LCM-EXPORT THE LCMLOGS & move it to matlab folder
                lcmlogexport_proc = subprocess.Popen(['sh',lcmlogexport_path,path_fly_no_fly,file_name.path], start_new_session=True)
                #wait for it to finish - kill it if it takes too long
                output, errors = lcmlogexport_proc.communicate()
                if not lcmlogexport_proc.returncode == 0:
                    print(f"Skipping non-file: {file_name.path}")



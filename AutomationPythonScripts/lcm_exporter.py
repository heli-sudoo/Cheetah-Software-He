#!/usr/bin/env python3

import os
import subprocess


LCMLOG_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM"
lcmlogexport_path = './../AutomationBashScripts/run_lcmexport.sh'


# Use os.scandir() to get an iterator of directory entries
with os.scandir(LCMLOG_env) as entries:
    for file_name in entries:
        if file_name.is_file():
            print(f"File: {file_name.path}")

            # LETS LCM-EXPORT THE LCMLOGS & move it to matlab folder
            lcmlogexport_proc = subprocess.Popen(['sh',lcmlogexport_path,file_name.name], start_new_session=True)
            #wait for it to finish - kill it if it takes too long
            output, errors = lcmlogexport_proc.communicate(timeout=180)
            if not lcmlogexport_proc.returncode == 0:
                print(f"Skipping non-file: {file_name.path}")



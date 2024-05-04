#!/usr/bin/env python3


import subprocess
import time
import signal
import threading
from tkinter import E



def manage_simulation():
    # Path to the Bash script
    ctrl_path = './../AutomationBashScripts/run_ctrl.sh'
    mhpc_path = './../AutomationBashScripts/run_mhpc.sh'


    while True:
        # Start the Bash scripts
        ctrl_proc = subprocess.Popen([ctrl_path], start_new_session=True)
        mhpc_proc = subprocess.Popen([mhpc_path], start_new_session=True)

        time.sleep(20)  # Run simulation for 10 seconds

        # Send SIGTERM to stop the simulation
        ctrl_proc.send_signal(signal.SIGTERM)
        ctrl_proc.send_signal(signal.SIGINT)
        ctrl_proc.send_signal(signal.SIGTERM)
        mhpc_proc.send_signal(signal.SIGINT)

        # Wait for the process to handle the signal and exit
        ctrl_proc.wait()
        mhpc_proc.wait()
        print("Simulation stopped.")

        # Optionally, pause before restarting or add condition to break the loop
        time.sleep(5)

def lcm_starter(): 
    lcmlog_path = './../AutomationBashScripts/run_lcmlog.sh'
    lcmexport_path = './../AutomationBashScripts/run_lcmexport.sh'

    
   
    i = 0
    while i < 3: 
        i = i + 1 
        filename = "here" + str(i)
        proc = subprocess.Popen(['sh',lcmlog_path,filename], start_new_session=True)
        # time.sleep(6)
        process = subprocess.Popen(['sh',lcmexport_path,filename], start_new_session=True)
        output, errors = process.communicate(timeout=10)

        if process.returncode == 0:
            print("Process completed successfully.")
            print("Output:", output)  # Decode from bytes to string if necessary
        else:
            print("Process failed with return code", process.returncode)
            print("Errors:", errors)  # Decode from bytes to string if necessary


        proc.terminate()
        process.terminate()
        proc.wait()
        process.wait()
        print("I got here")
        # LCMLOG.send_signal(signal.SIGTERM)
        # LCMLOG.send_signal(signal.SIGINT)
        # LCMLOG.wait()


if __name__ == "__main__":
    # manage_simulation()
    lcm_starter()

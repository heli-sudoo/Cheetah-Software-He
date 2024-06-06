#!/usr/bin/env python3


from os import fchdir
import subprocess
import time
import signal
import threading
from tkinter import E
import json




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

def gridder(min_force,max_force,frc_spacing): 
    import numpy as np

    # Create the meshgrid
    force_x = np.arange(min_force, max_force, frc_spacing)
    force_y = np.arange(min_force, max_force, frc_spacing)
    FORCE_X, FORCE_Y = np.meshgrid(force_x, force_y)

    number = 0
    for i in range(len(force_x)):
        for j in range(len(force_y)):   

            _frc_x = FORCE_X[j, i]
            _frc_y = FORCE_Y[j, i]
            print(f"Force applied will be (x,y) ({_frc_x},{_frc_y})")
            number = number + 1
    print("There will be %i simulations",number)


if __name__ == "__main__":
    # manage_simulation()
    # lcm_starter()

    import numpy as np

    min_force   = 0
    max_force   = 4.0
    frc_spacing = 0.2
    # gridder(min_force,max_force,frc_spacing )


    # Create the meshgrid
    force_x = np.arange(min_force, max_force, frc_spacing)
    force_y = np.arange(min_force, max_force, frc_spacing)
    FORCE_X, FORCE_Y = np.meshgrid(force_x, force_y)

    # print(FORCE_X[18])
    print(FORCE_Y[14:len(force_y)])
    # exit()
    
    ifas = 0
    # print(len(force_x))
    for i in range(14,len(force_x)):
        for j in range(0,len(force_y)):
            _frc_x = FORCE_X[i,j]
            _frc_y = FORCE_Y[i,j]
            ifas = ifas+1 
            print(f"i %i_frc_x %f _frc_y %f",j,_frc_x,_frc_y)


    print(f"number %i",ifas)



    with open('data_fly_May24') as f:
        d = json.load(f)
        print(len(d['rbtsf_avg']))

    exit()
    ifas = 0 

    for USE_FLY in range(2):
        print(USE_FLY)

    # for USE_FLY in range(2):
    #     print(f"Are we using the FLYWHEEL: {USE_FLY} ifas {ifas}")

    #     #------SIMPLE 2 RUNS STARTS HERE
    #     # _frc_x = 1.2
    #     # _frc_y = 1.2
    #     # i = 0
    #     # while i < 2 : 
    #     #         i = i + 1
    #     #---------ENDS HERE 

    #     #Activate the next few lines to do a full run 
    #     #------------------FROM HERE-------------------
    #     for iRepeats in range(3):
    #         for i in range(len(force_x)):
    #             for j in range(len(force_y)):   

    #                 _frc_x = FORCE_X[j, i]
    #                 _frc_y = FORCE_Y[j, i]
    #                 ifas = ifas+1 
    

    # print(f"ifas {ifas}")







    # lcmlog_path = './../AutomationBashScripts/run_lcmlog.sh'


    # path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/MC_SIM/NOFLY"
    # file_name = "lcmlog_fx"+str(1)+"_fy"+str(1)+"time"+str(1)+"nofly"

    # lcmlog_proc = subprocess.Popen(['sh',lcmlog_path,path_fly_no_fly,file_name], start_new_session=True)

    # time.sleep(10)

    # # Send SIGTERM to stop the simulation/CTRL/MHPC/SIMULATOR/LCMLOG
    # try:
    #     lcmlog_proc.send_signal(signal.SIGTERM)
    #     lcmlog_proc.send_signal(signal.SIGINT)
    #     lcmlog_proc.wait(timeout=10)  # Wait for 10 seconds for the process to terminate
    # except subprocess.TimeoutExpired:
    #     print("\n\n\n\nThis did not work??\n\n\n\n")
    #     # If the process doesn't terminate, kill it
    #     lcmlog_proc.send_signal(signal.SIGKILL)

    # file_path = './lcm_exporter.py'
    # with open(file_path, 'r') as file:
    #     exec(file.read())
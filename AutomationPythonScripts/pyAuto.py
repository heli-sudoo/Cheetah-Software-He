#!/usr/bin/env python3


from concurrent.futures import process
from pickle import FALSE
from re import T
from socket import timeout
from telnetlib import Telnet
import pyautogui
import subprocess
import time
import os
import atexit
import random
import signal
#Chose multiprocessing over threading.thread since multiprocessing.process has a terminate() to safely kills that process
import multiprocessing
import sys
import string

USE_THREADS = True
SIM_SPEED  = 0.8    
recognition_confidence = 0.75

processes = []
threadsss = []


#TODO LIST 
#In cleanup_process use lslocks | grep sim to release all files locks 
#Add failsafes if process, thread, simulator did not start
#Add a keyboardInterrupt
#remove all hardcoded directories

def cleanup_processes():
    print("\n There are %i procs in processes", len(processes))
    i = 0; 
    for proc in processes:
        i = i + 1
        if proc.poll() is None:           
            proc.terminate()
            try:
                proc.wait(timeout=3) 
                print(f"Process {proc.pid} terminated gracefully.")
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)  # Send SIGTERM to the entire process group
                print("Process did not terminate within 3 seconds. Forcing kill.")
                proc.kill()
                proc.wait(timeout=3) #to allow resources release -particularly flocks
        else: 
            print(f"Process {proc.pid} was already terminated with exit code {proc.returncode}")


def cleanup_threads(): 
    i = 0; 
    for _thread in threadsss:
        i = i + 1
        _thread.terminate()
        _thread.join()
        print(f"Thread {_thread.pid} terminated gracefully.")
        if _thread.is_alive():
            print("Thread did not terminate within 3 seconds. Forcing kill.")
            _thread.kill() # I also release resources
            _thread.join()

def restart_threadsm():
    print("Sending SIGUSR1 to restart CTRL and MHPC...")
    process.send_signal(signal.SIGUSR1)

def handle_exit(sig, frame):
    cleanup_processes()
    cleanup_threads()
    sys.exit(0)  # Exit after cleanup

# Register signal handlers
signal.signal(signal.SIGTERM, handle_exit)
signal.signal(signal.SIGINT, handle_exit)


def procs_alive():
    i=0
    for proc in processes:
        i = i + 1
        if proc.poll() is not None:    
            print("----------Process %i had already terminated----------",i)
            return False
    print("-----------ALL PROCESS ALIVE AND WELL------------")
    return True


def threads_alive():
    i=0
    for _thread in threadsss:
        i = i + 1
        if not _thread.is_alive():
            print("----------Thread %i had already terminated----------",i)
            return False
    print("-----------ALL THREADS ALIVE AND WELL------------")
    return True
    


# Register the cleanup function
atexit.register(cleanup_processes)
atexit.register(cleanup_threads)


# Path to the simulator executable
application_exec = r'/home/jnganga/Desktop/Cheetah-Software-He/build/sim/sim'
application_env  = r'/home/jnganga/Desktop/Cheetah-Software-He/build'

#Bash scripts to run CTRL,MHPC,LCMLOGGER
CTRL_env_run = '/home/jnganga/Desktop/Cheetah-Software-He/build/runCTRL.sh'
MHPC_env_run = '/home/jnganga/Desktop/Cheetah-Software-He/build/runMHPC.sh'
LCMLOG_env_run = '/home/jnganga/Desktop/Cheetah-Software-He/build/runLCMLOG.sh'
# Path to the Bash script
ctrl_path = './../AutomationBashScripts/run_ctrl.sh'
mhpc_path = './../AutomationBashScripts/run_mhpc.sh'
lcmlog_path = './../AutomationBashScripts/run_lcmlog.sh'
lcmlogexport_path = './../AutomationBashScripts/run_lcmexport.sh'

def run_LCMLOGGER(filename):
    LCMLOG = subprocess.Popen(['sh',LCMLOG_env_run,filename], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return LCMLOG


def wait(defaultTime=1):
    time.sleep(defaultTime)

def MHPC_thread_fn(print_out=False):
    print("-----------STARTING MHPC THREAD---------------")
    with subprocess.Popen(['/bin/bash',MHPC_env_run],bufsize=0, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,start_new_session=True) as proc:
        while True:
            output = proc.stdout.readline()
            if output == '' and proc.poll() is not None:
                break
            if output and print_out:
                print(output.strip())
        _, stderr = proc.communicate()
        if stderr:
            print("ERROR:", stderr.strip())

def CTRL_thread_fn(print_out=False):
    print("-----------STARTING CTRL THREAD---------------")
    with subprocess.Popen(['/bin/bash',CTRL_env_run],bufsize=0, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,start_new_session=True) as proc:
        while True:
            output = proc.stdout.readline()
            if output == '' and proc.poll() is not None:
                break
            if output and print_out:
                print(output.strip())
        # After the loop, print any remaining errors
        _, stderr = proc.communicate()
        if stderr:
            print("ERROR:", stderr.strip())


# MHPC_thread = threading.Thread(target=MHPC_thread_fn)
# CTRL_thread = threading.Thread(target=CTRL_thread_fn)


# MHPC_thread = multiprocessing.Process(target=MHPC_thread_fn)
# CTRL_thread = multiprocessing.Process(target=CTRL_thread_fn)

# threadsss.append(MHPC_thread)
# threadsss.append(CTRL_thread)

# MHPC_thread.start()
# CTRL_thread.start()

# print("MHPC_thread.returnCode")
# print(MHPC_thread.is_alive())
# wait(4)
# print(MHPC_thread.is_alive())
# wait(5)

# exit() 

def bringToFocus():
    sim_title_name = "SimControlPanel"
    # Bring the application window to the foreground using xdotool
    window_id = subprocess.getoutput(f"xdotool search --name {sim_title_name}")
    if window_id:
        os.system(f"xdotool windowactivate {window_id}")
    else:
        print("Window not found!")


i = 0 


#All the images
MC_selector_image = '/home/jnganga/Downloads/MC.png'
Simulator_selector_image = '/home/jnganga/Downloads/SIM.png'
START_selector_image = '/home/jnganga/Downloads/START.png'
USERC_REGION_selector_image = '/home/jnganga/Downloads/USERC_REGION.png'  
USERC_selector_image = '/home/jnganga/Downloads/USERC.png'  
CM_selector_image = '/home/jnganga/Downloads/CM.png'  
KICK_selector_image = '/home/jnganga/Downloads/KICK.png'  
FULLSEND_selector_image = '/home/jnganga/Downloads/FULLSEND.png'  
SIMSPEED_selector_image = '/home/jnganga/Downloads/SIMSPEED.png'  
STOP_selector_image= '/home/jnganga/Downloads/STOP.png'  


while i < 2 : 
    start_time = time.time()
    i = i + 1
    # Open the simulator
    simulator = subprocess.Popen(application_exec,cwd=application_env)
    processes.append(simulator)

    # wait sim to open
    wait(3)

    # Locate and click the 'Format' menu using an image file
    MC_menu_position = pyautogui.locateCenterOnScreen(MC_selector_image, confidence=recognition_confidence)
    SIM_menu_position = pyautogui.locateCenterOnScreen(Simulator_selector_image, confidence=recognition_confidence)
    START_menu_position = pyautogui.locateCenterOnScreen(START_selector_image, confidence=recognition_confidence)
    # USERC_REGION_menu_position = pyautogui.locateOnScreen(USERC_REGION_selector_image, confidence=recognition_confidence)
    # USERC_menu_position = pyautogui.locateCenterOnScreen(USERC_selector_image, confidence=recognition_confidence,region =USERC_REGION_menu_position)
    # CM_menu_position = pyautogui.locateCenterOnScreen(CM_selector_image, confidence=recognition_confidence)
    # KICK_menu_position = pyautogui.locateCenterOnScreen(KICK_selector_image, confidence=recognition_confidence)
    # FULLSEND_menu_position = pyautogui.locateCenterOnScreen(FULLSEND_selector_image, confidence=recognition_confidence)
    #SLOW DOWN THE SIMULATION
 
    if MC_menu_position is None:
        print("\n------SIMULATOR NEVER STARTED-------\n")

        ctrl_proc.send_signal(signal.SIGTERM)
        ctrl_proc.send_signal(signal.SIGINT)
        ctrl_proc.wait()

        mhpc_proc.send_signal(signal.SIGTERM)
        mhpc_proc.send_signal(signal.SIGINT)
        mhpc_proc.wait()

        simulator.send_signal(signal.SIGTERM)
        simulator.send_signal(signal.SIGINT)
        simulator.wait()

        continue
        

    pyautogui.click(MC_menu_position)
    wait() 

    pyautogui.click(SIM_menu_position)
    wait() 

    pyautogui.click(START_menu_position)
    wait() 

    bringToFocus()  
    wait(3) 

    USERC_menu_position = pyautogui.locateCenterOnScreen(USERC_selector_image, confidence=recognition_confidence) 

    #STOP USING THE RC CONTROLLER 
    pyautogui.click( USERC_menu_position.x + 100, USERC_menu_position.y + 10 )
    pyautogui.typewrite('0', interval=0.05)
    pyautogui.press('enter') 
    wait()

    # Start the CTRL and MHPC scripts
    ctrl_proc = subprocess.Popen([ctrl_path], start_new_session=True)
    mhpc_proc = subprocess.Popen([mhpc_path], start_new_session=True)

    #STAND UP
    CM_menu_position = pyautogui.locateCenterOnScreen(CM_selector_image, confidence=recognition_confidence)
    pyautogui.click(CM_menu_position)
    pyautogui.typewrite('1', interval=0.05)
    pyautogui.press('enter') 
    #wait for the robot to stand up
    wait(2)

    #SLOW DOWN THE SIMULATION
    if SIM_SPEED < 0.99:
        print("------THE SIMULATION IS SLOWED DOWN--------------")
    SIMSPEED_menu_position = pyautogui.locateCenterOnScreen(SIMSPEED_selector_image, confidence=recognition_confidence)
    pyautogui.click(SIMSPEED_menu_position.x,SIMSPEED_menu_position.y + 50)
    pyautogui.typewrite(str(SIM_SPEED), interval=0.05)
    pyautogui.press('enter') 

    #LETS GET A BUNCH OF LOCATIONS 
    KICK_menu_position = pyautogui.locateCenterOnScreen(KICK_selector_image, confidence=recognition_confidence)
    FULLSEND_menu_position = pyautogui.locateCenterOnScreen(FULLSEND_selector_image, confidence=recognition_confidence)
    STOP_menu_position = pyautogui.locateCenterOnScreen(STOP_selector_image, confidence=recognition_confidence) 

    #Magnitude of the forces
    force_x = 2
    force_y = 1

    #ADD RANDOM KICK IN X DIRECTION
    pyautogui.click(KICK_menu_position.x, KICK_menu_position.y )
    pyautogui.press('delete') 
    pyautogui.typewrite(str(force_x), interval=0.05)
    pyautogui.press('enter') 

    #ADD RANDOM KICK IN Y DIRECTION
    pyautogui.click(KICK_menu_position.x, KICK_menu_position.y + 25)
    pyautogui.press('delete') 
    pyautogui.typewrite(str(force_y), interval=0.05)
    pyautogui.press('enter') 

    time_of_kick = random.uniform(2, 7)
    #Avoid naming conflicts -- add 3 random chars at the end of file_name
    random_string = ''.join(random.choices(string.ascii_letters + string.digits, k=3))
    


    file_name = "lcmlog_fx"+str(force_x)+"_fy"+str(force_y)+" "+random_string
    lcmlog_proc = subprocess.Popen(['sh',lcmlog_path,file_name], start_new_session=True)


    # START LOCOMOTION
    pyautogui.click(CM_menu_position)
    pyautogui.press('delete') 
    pyautogui.typewrite('2', interval=0.05)
    pyautogui.press('enter') 
    wait()

    print("Robot will be kicked at %f seconds with %f in x and %f in y dir",time_of_kick,force_x,force_y)
    wait(time_of_kick) 
    pyautogui.click(FULLSEND_menu_position)
    wait(12 - time_of_kick) 



    # Send SIGTERM to stop the simulation/CTRL/MHPC/SIMULATOR/LCMLOG
    lcmlog_proc.terminate()
    lcmlog_proc.wait()
    
    ctrl_proc.send_signal(signal.SIGTERM)
    ctrl_proc.send_signal(signal.SIGINT)
    ctrl_proc.wait()

    mhpc_proc.send_signal(signal.SIGTERM)
    mhpc_proc.send_signal(signal.SIGINT)
    mhpc_proc.wait()

    simulator.send_signal(signal.SIGTERM)
    simulator.send_signal(signal.SIGINT)
    simulator.wait()

    # LETS LCM-EXPORT THE LCMLOGS & move it to matlab folder
    # lcmlogexport_proc = subprocess.Popen(['sh',lcmlogexport_path,file_name], start_new_session=True)
    # #wait for it to finish - kill it if it takes too long
    # output, errors = lcmlogexport_proc.communicate(timeout=15)
    # if process.returncode == 0:
    #     print("\nLCM EXPORT OF %s Failed",file_name)

    #LCMEXPORT TAKES LONG TIME, SEE LCMEXPORTER.PY FOR SOLUTION
    


    elapsed = time.time() - start_time

    print("Each RUN Takes %f seconds",elapsed)
    #Breathe
    wait(3)



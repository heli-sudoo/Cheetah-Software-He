
# This code is ran with signal scope
# To run this code
# conda activate signal-scope
# signal-scope scripts/hardware_play/monitor_torque.py

import time
# This reads every .py file in the given directory and catalogs
# everything that is an lcm type python definition file.
findLCMTypes(os.path.expanduser('lcm_types/python'))


# This looks for lcm packages.  An lcm package is a directory
# that contains an __init__.py file and lcm type python files.
findLCMModules(os.path.expanduser('lcm_types/python'))

start_time = time.time()
def plot_torque(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.tau_est

def plot_joint_velocity(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.qd

def plot_vWorld(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.vWorld

def plot_omega(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.omegaBody

def plot_height(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.p

def cmd_torque(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.torque[0]

def cmd_contact(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.contacts[0]

# Plot one sequence
p = addPlot()
addSignalFunctions("MHPC_COMMAND", cmd_torque, [3,4,5], plot = p)

ptau = addPlot()
addSignalFunctions('leg_control_data', plot_torque, [0,1,2], plot=ptau)

pqd = addPlot()
addSignalFunctions('leg_control_data', plot_joint_velocity, [0,1,2], plot=pqd)

pc = addPlot()
addSignalFunctions("MHPC_COMMAND", cmd_contact, [1], plot = pc)

# pVword = addPlot()
# addSignalFunctions('state_estimator', plot_vWorld, [0,1,2], plot=pVword)

# pomega = addPlot()
# addSignalFunctions('state_estimator', plot_omega, [0,1,2], plot=pomega)

# pheight = addPlot()
# addSignalFunctions('state_estimator', plot_height, [1,2], plot=pheight)

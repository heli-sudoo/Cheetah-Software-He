
import time
# This reads every .py file in the given directory and catalogs
# everything that is an lcm type python definition file.
findLCMTypes(os.path.expanduser('lcm-types/python'))


# This looks for lcm packages.  An lcm package is a directory
# that contains an __init__.py file and lcm type python files.
findLCMModules(os.path.expanduser('lcm-types/python'))

start_time = time.time()
def myFunction(msg):
    current_time = time.time()
    time_flight =  (current_time - start_time)*1e6
    return time_flight, msg.tau_est

pFront = addPlot()
addSignalFunctions('leg_control_data', myFunction, [0,1,2], plot=pFront)

pBack = addPlot()
addSignalFunctions('leg_control_data', myFunction, [6,7,8], plot=pBack)


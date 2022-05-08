
# This reads every .py file in the given directory and catalogs
# everything that is an lcm type python definition file.

findLCMTypes(os.path.expanduser('/home/heli/Research/Cheetah-Software-Revised/lcm-types/python'))


# This looks for lcm packages.  An lcm package is a directory
# that contains an __init__.py file and lcm type python files.
findLCMModules(os.path.expanduser('/home/heli/Research/Cheetah-Software-Revised/lcm-types/python'))


# plot contact sequence
pc_FR = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.contacts[0], plot=pc_FR)

pc_FL = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.contacts[1], plot=pc_FL)

pc_HR = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.contacts[2], plot=pc_HR)

pc_HL = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.contacts[3], plot=pc_HL)

setFormatOptions(curveStyle='points', timeWindow=1, pointSize=4)



# This reads every .py file in the given directory and catalogs
# everything that is an lcm type python definition file.

findLCMTypes(os.path.expanduser('/home/heli/Research/Cheetah-Software-Revised/lcm-types/python'))


# This looks for lcm packages.  An lcm package is a directory
# that contains an __init__.py file and lcm type python files.
findLCMModules(os.path.expanduser('/home/heli/Research/Cheetah-Software-Revised/lcm-types/python'))


# plot contact sequence
p_FR = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.qdummy_r[0], plot=p_FR)

p_FL = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.qdummy_r[3], plot=p_FL)

p_HR = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.qdummy_r[6], plot=p_HR)

p_HL = addPlot()
addSignalSequence("DEBUG_HKDMPC", msg.times, msg.qdummy_r[9], plot=p_HL)

setFormatOptions(curveStyle='points', timeWindow=1, pointSize=4)


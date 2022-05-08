import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import time
import lcm
import _thread
import copy

script_path = os.path.dirname(os.path.realpath(__file__))
lcmtype_path = script_path + "/../lcm-types/python"
sys.path.append(lcmtype_path)


from opt_sol_lcmt import opt_sol_lcmt

contacts = []
qdummys = []
N = 0
colors = ['r','g','b','y']

def update_plot(fig, lines):
    """ visualize the foot placements """
    """ cs: sequence of contacts in list
        qdummy: sequence of qdummies in list"""    
    
    pfs = [{'x':[], 'y':[]} for l in range(4)]
    for k in range(N):
        # get the contact status and foot placement at time k
        c = contacts[k]
        qdummy = qdummys[k]
        for l in range(4):
            if c[l]==1:
                if len(pfs[l]['x'])==0 and len(pfs[l]['y'])==0:
                    pfs[l]['x'].append(qdummy[3*l])
                    pfs[l]['y'].append(qdummy[3*l+1])
    for l in range(4):
        lines[l].set_xdata(np.asarray(pfs[l]['x']))
        lines[l].set_ydata(np.asarray(pfs[l]['y']))
    fig.canvas.draw()
    fig.canvas.flush_events()


def my_handler(channel, data):
    msg = opt_sol_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   length    = %s" % str(msg.N))
    print("")
    global contacts
    global qdummys
    global N
    contacts = msg.contacts
    qdummys = msg.qdummy
    N = msg.N

def lcm_thread(lc):
    while True:
        lc.handle()
    
lc = lcm.LCM()
subscription = lc.subscribe("debug_foot", my_handler)
plt.ion()

try:
    _thread.start_new_thread(lcm_thread, (lc,))
    lines = []
    fig, ax = plt.subplots(figsize=(8,6))
    for l in range(4):
        line, = ax.plot(0, 0, colors[l]+'o')
        ax.axis([-0.5, 0.5, -0.5, 0.5])
        lines.append(line)
    ax.set_xlabel('px')
    ax.set_ylabel('py')
    ax.legend(lines,['FR','FL','HR','HL'])
    while True:
        update_plot(fig, lines)
        
except KeyboardInterrupt:
    pass
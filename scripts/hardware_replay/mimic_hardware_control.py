
import os
import sys
cwd = os.getcwd()
sys.path.append(cwd + '/lcm-types/python/')

import lcm

from control_parameter_request_lcmt import control_parameter_request_lcmt
from control_parameter_respones_lcmt import control_parameter_respones_lcmt


response_msg = control_parameter_respones_lcmt()

def parameter_request_handler(channel, data):
    msg = control_parameter_request_lcmt.decode(data)    
    response_msg.name = msg.name
    response_msg.parameterKind = msg.parameterKind
    response_msg.requestKind = msg.requestKind
    response_msg.requestNumber = msg.requestNumber


lc = lcm.LCM()
subscription = lc.subscribe("interface_request", parameter_request_handler)
try:
    while True:        
        print("wainting for request")
        lc.handle()        
        # break  
        lc.publish("interface_response", response_msg.encode())
        print("published reponse messages")
                      
except KeyboardInterrupt:
    pass
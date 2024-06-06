#!/usr/bin/env python3

from hashlib import new
import os
import subprocess
from xml.dom.pulldom import default_bufsize
import scipy.io
import numpy as np 
import matplotlib.pyplot as plt
from collections import defaultdict
import json
import itertools

import time


# MATLAB_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM/MatlabData"



# path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/MC_SIM_MAY8_MOREDATA/FLY/MatlabData"
# rFile  = "lcmlog_fx3_8000000000000003_fy3_8000000000000003_time3_3825659099325223_flyZAH.mat"

# data = scipy.io.loadmat(path_fly_no_fly + "/" + rFile)


# GRFs = data["simulator_state"]["f_foot"][0][0]

# Fx = np.max(np.abs(GRFs[:,:,0]))
# Fy = np.max(np.abs(GRFs[:,:,1]))
# Fz = np.max(np.abs(GRFs[:,:,2]))

# print(f"Fx.shape {Fx}")
# print(f"Fy.shape {Fy}")
# print(f"Fz.shape {Fz}")


# time.sleep(5)

# exit()



USE_FLY_OPTIONS = {True,False}

for USE_FLY in USE_FLY_OPTIONS:

    if USE_FLY:
        path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/MC_SIM_MAY8_MOREDATA/FLY/MatlabData"
    else:
        path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/MC_SIM_MAY8_MOREDATA/NOFLY/MatlabData"
            
    force_x    = []
    force_y    = []
    rbtsafe    = []
    timekicked = []
    Fx_max,Fy_max,Fz_max = [],[],[]

    NOT_PROCESSED = []

    # NO NEED TO READ FOR NOW!!!
    with os.scandir(path_fly_no_fly) as entries:
        for file_name in entries:
        # for file_name in itertools.islice(entries, 10):

            if file_name.is_file():
                print(f"---------Processing LCM File: {file_name.name}")

                try:
                    data = scipy.io.loadmat(file_name.path)

                    # print(f"data {data}")
                    rbtFailed = data["MHPC_DATA"]["robotFailed"][0][0]
                    GRFs = data["simulator_state"]["f_foot"][0][0]


                    # print(f"rbtFailed {rbtFailed}")

                    file_name_split = file_name.name.split(" ")
                    iCare = file_name_split[0].split("_")

                    print(f"file_name_split {file_name_split } ")
                    print(f"iCare { iCare} ")
               

                    frc_x        = float(iCare[1][-1] + "." + iCare[2] )  
                    frc_y        = float(iCare[3][-1] + "." + iCare[4] )  
                    time_of_kick = float(iCare[5][-1] + "." + iCare[6] )

                    # print(f" frc_x {frc_x } ")
                    # print(f" frc_y { frc_y} ")
                    print(f"time_of_kick {time_of_kick}")

                    robotFailedSum = np.sum(rbtFailed)
                    if robotFailedSum > 10: 
                        robotFailed = 1
                    else:
                        robotFailed = 0

                    
                    Fx,Fy,Fz =  0.0, 0.0, 0.0
                    if robotFailed == 0: 
                        Fx = np.max(np.abs(GRFs[:,:,0]))
                        Fy = np.max(np.abs(GRFs[:,:,1]))
                        Fz = np.max(np.abs(GRFs[:,:,2]))
                    

                    Fx_max.append([Fx])
                    Fy_max.append([Fy])
                    Fz_max.append([Fz])


                    # print(f" robotFailed { robotFailed} ")

                    force_x.append(frc_x)
                    force_y.append(frc_y)
                    rbtsafe.append(robotFailed)
                    timekicked.append(time_of_kick)

                    


                except:
                    print(f"---------FAILED TO Process LCM File: {file_name.name}")
                    NOT_PROCESSED.append(file_name)
                    exit()


    print(f"I was not able to process {len(NOT_PROCESSED)} files")
    for _item in NOT_PROCESSED:
        print(f"\n....Did not process {_item.name} " )



    # force_x.append(0)
    # force_y.append(0)
    # rbtsafe.append(1)


    # force_x.append(0)
    # force_y.append(0)
    # rbtsafe.append(1)

    # force_x.append(0)
    # force_y.append(0)
    # rbtsafe.append(1)

    #we picking out duplicates frcx and frcy
    frc_dict = defaultdict(list)
    for frc_x, frc_y, rbtsf in zip(force_x, force_y, rbtsafe):
        frc_dict[(frc_x, frc_y)].append(rbtsf)

    Fmax_dict = defaultdict(list)
    for frc_x, frc_y, fx,fy,fz in zip(force_x, force_y, Fx_max,Fy_max,Fy_max):
        Fmax_dict[(frc_x, frc_y)].append([fx,fy,fz])


    # print("Fmax_dict")
    # print(Fmax_dict)

    

    # Average rbtsafety
    frc_x_unique, frc_y_unique, rbtsf_avg = [], [], []
    for (frc_x, frc_y), rbtsf_list in frc_dict.items():
        frc_x_unique.append(frc_x)
        frc_y_unique.append(frc_y)
        rbtsf_avg.append(np.mean(rbtsf_list))
        # print(f"\n x {frc_x} y {frc_y} z {rbtsf_list}")
    
    
    data = {
            # "dict_no_avg":  dict(frc_dict), 
            "frc_x_unique": frc_x_unique,
            "frc_y_unique": frc_y_unique,
            "rbtsf_avg":    rbtsf_avg
            # "timekicked":   rbtsf_avg.tolist()
        }


    frcmax_x_unique, frcmax_y_unique  = [], []
    Fx_maxed, Fy_maxed, Fz_maxed = [], [], []
    for (frc_x, frc_y), Frcs in Fmax_dict.items():
        frcmax_x_unique.append(frc_x)
        frcmax_y_unique.append(frc_y)


        Fx_maxed.append(np.mean(Frcs[0]))
        Fy_maxed.append(np.mean(Frcs[1]))
        Fz_maxed.append(np.mean(Frcs[2]))
        # print(f"\n x {frc_x} y {frc_y} z {rbtsf_list}")

    dataFrcs =   {   
        "frc_x_unique": frc_x_unique,
        "frc_y_unique": frc_y_unique,
        "Fx_max":    Fx_maxed,
        "Fy_max":    Fy_maxed,
        "Fz_max":    Fz_maxed
    }


    if USE_FLY: 
        data_file_path = "data_fly_May8"
        fmax_file_path =  "data_FLY_Fmax_May8"
    else: 
        data_file_path = "data_NOfly_May8"
        fmax_file_path =  "data_NOFLY_Fmax_May8"


    with open(data_file_path, 'w') as file:
        json.dump(data, file)
        # scipy.io.savemat(file+".mat",data)

    with open(fmax_file_path, 'w') as file:
        json.dump(dataFrcs, file)

    print(f" forces in x unique is  {frc_x_unique}   ")
    print(f" forces in y unique is  {frc_y_unique}   ")
    print(f" robotsafe avg is  {rbtsf_avg}   ")


    plt.figure()
    contour = plt.tricontourf(frc_x_unique, frc_y_unique, rbtsf_avg, levels=3, cmap='viridis')
    cbar = plt.colorbar(contour)
    cbar.set_ticks([np.min(rbtsafe), np.max(rbtsafe)])
    cbar.set_ticklabels(['Safe', 'Not Safe'])
    plt.xlabel('Velocity Change +X')
    plt.ylabel('Velocity Change +Y')
    # plt.show()

    if USE_FLY:
        file_name = "May8 RobotSafetyFlywheel, level 3"
        titled    = "May8 Robot Safety - Flywheel"
    else:
        file_name = "May8 RobotSafetyNOFlywheel, level 3"
        titled    = "May8 Robot Safety - NO Flywheel"
        
    plt.title(titled)
    plt.savefig(file_name+".png",dpi=600) 
    plt.savefig(file_name+".svg") 
    plt.savefig(file_name+".pdf",dpi=600) 


    plt.figure()
    contour = plt.tricontourf(frc_x_unique, frc_y_unique, rbtsf_avg, levels=500, cmap='viridis')
    cbar = plt.colorbar(contour)
    cbar.set_ticks([np.min(rbtsafe), np.max(rbtsafe)])
    cbar.set_ticklabels(['Safe', 'Not Safe'])
    # plt.title('RobotSafety')
    plt.xlabel('Velocity Change +X')
    plt.ylabel('Velocity Change +Y')
    # plt.show()

    if USE_FLY:
        file_name = "May8 RobotSafetyFlywheel, level 500"
        titled    = "May8 Robot Safety - Flywheel"
    else:
        file_name = "May8 RobotSafetyNOFlywheel, level 500"
        titled    = "May8 Robot Safety - NO Flywheel"
        
    plt.title(titled)
    plt.savefig(file_name+".png",dpi=600) 
    plt.savefig(file_name+".svg") 
    plt.savefig(file_name+".pdf",dpi=600) 


    plt.figure()
    contour = plt.tricontourf(frc_x_unique, frc_y_unique, rbtsf_avg, levels=5000, cmap='viridis')
    cbar = plt.colorbar(contour)
    cbar.set_ticks([np.min(rbtsafe),0.5])
    cbar.set_ticklabels(['Safe', 'Not Safe'])
    # plt.title('RobotSafety')
    plt.xlabel('Velocity Change +X')
    plt.ylabel('Velocity Change +Y')
    # plt.show()

    if USE_FLY:
        file_name = "May8 RobotSafetyFlywheel, level 5000"
        titled    = "May8 Robot Safety - Flywheel"
    else:
        file_name = "May8 RobotSafetyNOFlywheel, level 5000"
        titled    = "May8 Robot Safety - NO Flywheel"
        
    plt.title(titled)
    plt.savefig(file_name+".png",dpi=600) 
    plt.savefig(file_name+".svg") 
    plt.savefig(file_name+".pdf",dpi=600) 

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # surf = ax.plot_surface(frc_x_unique, frc_y_unique, rbtsf_avg, cmap='viridis')

    # ax.set_xlabel('Velocity Change +X')
    # ax.set_ylabel('Velocity Change +X')
    # ax.set_zlabel('Robot Safety')

    # if USE_FLY:
    #     file_name = "RobotSafetyFlywheel, surf"
    #     titled    = "Robot Safety - Flywheel"
    # else:
    #     file_name = "RobotSafetyNOFlywheel, surf"
    #     titled    = "Robot Safety - NO Flywheel"

    # ax.set_title(titled) 
    # fig.colorbar(surf, shrink=0.5, aspect=5)

    # plt.savefig(file_name+".png",dpi=600) 
    # plt.savefig(file_name+".svg") 
    # plt.savefig(file_name+".pdf",dpi=600) 


    if USE_FLY:
        print("\n\nDone Processing files for FLY data")
    else:
        print("\n\nDone Processing files for NOFLY data")



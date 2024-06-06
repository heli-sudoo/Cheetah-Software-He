#!/usr/bin/env python3

from hashlib import new
import os
import subprocess
import scipy.io
import numpy as np 
import matplotlib.pyplot as plt
from collections import defaultdict
import json
import itertools



# MATLAB_env="/media/jnganga/Internal Storage1/jnganga/MC_SIM/MatlabData"


USE_FLY_OPTIONS = {True,False}

for USE_FLY in USE_FLY_OPTIONS:

    if USE_FLY:
        path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/MC_SIM_MAY24_MOREDATA/FLY/MatlabData/"
    else:
        path_fly_no_fly ="/media/jnganga/Internal Storage1/jnganga/MC_SIM_MAY24_MOREDATA/NOFLY/MatlabData/"
            
    force_x    = []
    force_y    = []
    rbtsafe    = []
    timekicked = []

    NOT_PROCESSED = []

    # NO NEED TO READ FOR NOW!!!
    with os.scandir(path_fly_no_fly) as entries:
        # for file_name in itertools.islice(entries,20):
        for file_name in entries:
            if file_name.is_file():
                print(f"---------Processing LCM File: {file_name.name}")

                try:
                    data = scipy.io.loadmat(file_name.path)

                    # # print(f"data {data}")
                    rbtFailed = data["MHPC_DATA"]["robotFailed"][0][0]

                    # print(f"rbtFailed {rbtFailed}")

                    file_name_split = file_name.name.split(" ")
                    iCare = file_name_split[0].split("_")

                    # print(f"file_name_split {file_name_split } ")
                    # print(f"iCare { iCare} ")


                    frc_x        = float(iCare[1][-1] + "." + iCare[2] )  
                    frc_y        = float(iCare[3][-1] + "." + iCare[4] )  


                    # frc_x        = float(iCare[1][2:] )  
                    # frc_y        = float(iCare[2][2:] )  


                    time_of_kick = float(iCare[5][-1] + "." + iCare[6] )

                    # print(f" frc_x {frc_x } ")
                    # print(f" frc_y { frc_y} ")
                    # print(f"time_of_kick {time_of_kick}")

                    robotFailedSum = np.sum(rbtFailed)
                    if robotFailedSum > 10: 
                        robotFailed = 1
                    else:
                        robotFailed = 0

                    # print(f" robotFailed { robotFailed} ")

                    force_x.append(frc_x)
                    force_y.append(frc_y)
                    rbtsafe.append(robotFailed)
                    timekicked.append(time_of_kick)
                except:
                    print(f"---------FAILED TO Process LCM File: {file_name.name}")
                    NOT_PROCESSED.append(file_name)


    print(f"I was not able to process {len(NOT_PROCESSED)} files")
    for _item in NOT_PROCESSED:
        print(f"\n....Did not process {_item.name} " )

    # print(force_x)
    # print(force_y)

    # frc_dict = defaultdict(list)
    # for frc_x, frc_y in zip(force_x, force_y):
    #     frc_dict[(frc_x)].append(frc_y)

    # print(frc_dict.keys())


    # frc_x_unique, frc_y_unique = [], [], 
    # for (frc_x, frc_y) in frc_dict.items():
    #     frc_x_unique.append(frc_x)
    #     frc_y_unique.append(frc_y)

    # print("unique")
    # print(frc_x_unique)
    # print(frc_y_unique)
    # exit()

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

    # print("frc_dict")
    # print(frc_dict)

    

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

    if USE_FLY: 
        data_file_path = "data_fly_May24"
    else: 
        data_file_path = "data_NOfly_May24"

    with open(data_file_path, 'w') as file:
        json.dump(data, file)
        # scipy.io.savemat(file+".mat",data)


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
        file_name = "May24 RobotSafetyFlywheel, level 3"
        titled    = "May24 Robot Safety - Flywheel"
    else:
        file_name = "May24 RobotSafetyNOFlywheel, level 3"
        titled    = "May24 Robot Safety - NO Flywheel"
        
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
        file_name = "May24 RobotSafetyFlywheel, level 500"
        titled    = "May24 Robot Safety - Flywheel"
    else:
        file_name = "May24 RobotSafetyNOFlywheel, level 500"
        titled    = "May24 Robot Safety - NO Flywheel"
        
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
        file_name = "May24 RobotSafetyFlywheel, level 5000"
        titled    = "May24 Robot Safety - Flywheel"
    else:
        file_name = "May24 RobotSafetyNOFlywheel, level 5000"
        titled    = "May24 Robot Safety - NO Flywheel"
        
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



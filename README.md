# SIR_2022 at the Network Ingeneering Departement of the UPC 

The goal is to compare the efficiency between wifi Halow communication and simple wifi communication to build a multi-hop network using drones.

We use droneKit and droneKit SITL to simulate the drones 

## Setup the environment 
You first have to download the image disk of the virutal machine you have to use to run your tests.

A mission planner connection list is generated in /mnt/hgfs/connect/ , our advise is to use this folder as shared folder with the host. Doing so will facilitate the access to the file for giving it to mission planner 


## Launching the simulation

The code is meant to be launch by running GCS-T.py
In GCS-T.py, global varaibles at the beggining of the code allows you to easily change some parameters, like the path to the shared folder, the number of drones to use in the simulation, if the drones have an Halow interface or if you want to print all the logs in terminal 


## The mission flow
All the drones takeoff at the same position (considered as the Ground Control Station position too) and will follow their mission until one of them detects (pass near ) a Point Of Interest. Then it will try to inform the GCS by sending it the "video" of this POI.
* If the GCS is near enough (under 100 meters), the "video" is directly sended by wifi. 
* If the GCS is too away (more than 100 meters) :
   * if the drone has an Halow interface, it will send its position to the GCS using Halow, and the GCS will determine the optimal positions for other drones in order to build the multi-hop network with lesser drones and it will send to each concerned drone that position using Halow. When they're all aligned, the video will be sended by the drone that detected the POI.
   * if the drone doesn't have an halow interface, it will dertermine the network map of all accessible drones, then it will ask for their positions and determine the needed position to build the multi-hop network with lesser drones, then it will send to the concerned drones the position where they have to go, once they're aligned, it will send to the GCS the "video" and wait for it's awnser before ordering to all drones to continue their missions.

Repo for the work of Thomas CADALEN and Paul CHOCHILLON during their intership at the University Polytecnica de Catalunya 

<a href="https://www.upc.edu/en"><img src="https://github.com/ThomasCdln/SIR_2022/blob/4c60a2bdb55a00e7e470eadb8bddc0189484c948/images/logo_upc.png" width=140 align=left /><a href="https://polytech.univ-amu.fr/"><img src="https://github.com/ThomasCdln/SIR_2022/blob/64e1d623ef57af6fbaae856be829bfed9ab05bec/images/logo_PM.png" width=450 align=right/></a>



# THÉ tutorial of how to launch everything for the in the flight arena

## Introduction
This is a tutorial of how to make everything ready to let a drone fly in the test arena and run your scripts. It is important to know that this tutorial is not complete, nor 100% right. Some things can also be a bit vague or misleading.

You will need 3 different terminal screens, so using terminator app is adviced. You can copy an tab with `docker exec -it ...... bash`

## Table of Contents
1. [Safety](#safety)
2. [Setting up WiFi and ID](#setting-up-wifi-and-id)
3. [Going into Agilicious@raspberrypi](#going-into-agiliciousraspberrypi)
4. [Going into Agilicious environment](#going-into-agilicious-environment)
5. [Random stuff I wrote down but don't know where to put haha](#random-stuff-i-wrote-down-but-dont-know-where-to-put-haha)

## Safety
For using the test arena there are some safety precautions and checklists that need to be taken into account. 

For the drone(s) these are:
1. Needs to have 4 motion capture balls attached (the bar also)
2. Test if propellers are well attached (and intact)
3. Make sure raspberrypi is connected. Green light means good, red light means bad. In that case you can press the button or reset.
4. Battery cables need to be on the top side (makes sound when you connect the battery to the drone)
5. Check if SD card is well inserted

For the tracking system these are:
1. Check if falcon1/falcon2 checkboxes are active
2. Check if tracking system is live (spacebar can pause tracking system)

For the safety precautions, the following examples are the most important:
1. Wear safety glasses if you are planning to fly
2. Close black cage curtains (also the one to enter the cage)

For all the safety precautions there is a github page: [https://github.com/cor-mobile-robotics/lab-wiki]

## Setting up WiFi and ID
First we need to make sure that a connection can be made with the drone. Therefore we need to do the following things:
1. Connect to the same WiFi as the drone
  mrl-wifi is the correct network, and the password is on the back of the router
2. Determine the IP address of your own computer, you will need this later

## Going into Agilicious@raspberrypi
We need to go into the docker environment of the raspberrypi. This is done by:
1. We go in the falcon@raspberrypi environment from the agiclean folder with the following code: `ssh falcon@raspberry3` or `ssh falcon@raspberry4`
2. You will need a password, which is: `falcon`
3. Git clone the desired repository
4. Go into the docker environment agilicious@raspberrypi by the following code: `agilicious`
5. Time to build the catkin from the catkin folder, but by the following code: `catkin build agiros`
6. Source the `sourceros`
7. Source your own script here as well

NOTE: Specify `quad_name:=falcon3` when launching (or `quad_name1:=falcon4`)

## Going into Agilicious environment
We need to go in a docker on our own computer as well. Follow these steps:
1. You need to go in docker environment
2. Perform `catkin build` like normal and source it
3. You can check your laptop ID with: `ifconfig`
4. Create a `launch_script` file with drone ID and computer ID for each drone and your PC and change the ID for the master and slave as desired
5. Change the computer ID to the one you found while performing the `ifconfig` command
6. Source the `launch_script` file 

## Random stuff I wrote down but don't know where to put haha
1. We need to tell ROS who is master and who is slave. 1 drone master and 1 drone slave is possible, but so is making the computer the master. But when the computer crashes drones will go to Narnia. For this you can make a file where you define the master and slave and their IDs.
3. `sudo docker system prune -a -f` (needed to write this down from a master's student)
4. When placing the drone, the battery connection side needs to face desks

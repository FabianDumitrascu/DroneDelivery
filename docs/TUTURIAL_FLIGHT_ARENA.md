# The tutorial of how to launch everything for the in the flight arena

## Table of Content
1. [Setting up WiFi and ID](##Setting-up-WiFi-and-ID)
2. [Going into agilicious@rasberrypi](##Going-into-agilicious@rasberrypi)
3. [Going to docker on your own computer](##Going-to-docker- on your own computer)

## Introduction
There is a github page for all the safety instructions. Such as controlling the drone and the flight arena. 

Having said that, it is important to know that this tutorial is not complete. Some things can be a bit vague or misleading.

## Setting up WiFi and ID
First we need to make sure that a connection can be made with the drone. Therefore we need to do the following things:
1. Connect to the same WiFi as the drone
  mrl-wifi is the correct network, and the password is on the back of the router
2. Determine the IP adress of your own computer, you will need this later

## Going into Agilicious@rasberrypi
We need to go into the docker environment of the rasberrypi. This is done by:
1. We go in the falcon@rasberrypi environment with the following code: `ssh falcon@#falconID`
2. You will need a password, wich is: `rasberrypi`
3. Git clone the desired repository
4. Go into the docker environment by the standard way
5. Time to build the catkin, but by the following code: `catkin build agiros`
6. Source the `source/devel/setup.bash`
7. Source something else lol

## Going to docker on your own computer
We need to go in a docker on our own computer as well. Follow these steps:
1. 


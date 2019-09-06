# What is RoCS?
RoCS (Robotics and Cognitive Systems) is a framework to support the software development for autonomous robots. The framework is based on the well-known IBM Autonomic Computing reference architecture (also known as MAPE-K). 

# Why should I use RoCS?

Some important principles what we belive to be advantages of adopting the RoCS framework:
* RoCS intends to be general and multiplatform, forstering reuse across different robot models. 
* RoCS core architecture is simple and use few components that communicate through recognized patterns. 
* RoCS implements the hybrid robotics paradigm, making possible to use deliberative and reactive behaviors.

# Getting Started

## Requirements and Dependencies
- VREP
- a c++ compiler
- make and cmake

## Setting up
These instructions were tested in Arch Linux, Fedora 23 and MacOS.
- To compile, execute the compile_pioneer_and_robotnik.sh script
- Change the remoteApiConnections.txt file in VREP directory and make sure it has the following lines:

```
portIndex1_port             = 19997
portIndex1_debug            = false
portIndex1_syncSimTrigger   = true

portIndex2_port             = 19998
portIndex2_debug            = false
portIndex2_syncSimTrigger   = true
```

- Open VREP and load the scene ./vrep/scene_02.ttt
- Start the simulation pressing the play button
- To execute the code for the Pioneer run ./pioneer_wall_tests/cmake-build-debug/pioneer_xwalk
- To execute the code for the Robotnik run ./robotnik_follow/cmake-build-debug/robotnik_follow



# Interested in contributing to RoCS?
Thanks for the interest and please read the [Contributing](https://github.com/larocs/RoCS/blob/master/CONTRIBUTING.md) recommendations.

# Authors
Leonardo de Oliveira Ramos
Gabriel Divino
Leonardo Montecchi
Breno Bernard Nicolau de França
Esther Colombini


S.M.O.R.E.S. Simulator
=================

Author: Yunkai (Edward) Cui, Modlab, University of Pennsylvania<br>
Co-author: Adam Trofa, Cornell University<br>
Co-author: Gangyuan (Jim) Jing, Cornell University<br>
Co-author: Tarik Tosun, University of Pennsylvania

Started Date: 08/2013

Last Edti: 12/24/2013 


Introduction
----------------------------------------
The SMORES simulator aims to model the capabilities of the SMORES modular robot platform, allowing researchers to rapidly assess the capabilities of different hardware configurations and develop the algorithms required to exercise the powerful flexibility available to them. In addition to providing accurate dynamics modeling of a given robotic configuration, the simulator aims to model the reconfiguration and sensing capabilities of the SMORES modules as well.

The core physics and control of the simulator are currently being developed at Modlab at Penn using the simulation software Gazebo. The sensor add-ons that can be used to provide individual modules with specific sensing capabilities are being developed by Autonomous Systems Lab at Cornell. Both groups are cooperating on a NSF funded project.

For More information about the Gazebo simulation engine, please visit [GAZEBOsim](http://gazebosim.org/)

For more information about Modlab at Penn, please visit [Modlab](http://modlabupenn.org/)

For more information about Autonomous Systems Lab at Cornell, please visit [Autonomous Systems Lab](http://cornell-asl.org/wiki/index.php?title=Main_Page)

Installation and Quick Start
----------------------------
**Gazebo** (API level 4.0)

To use the simulator, you will first need to install Gazebo. Both precompiled binaries and source codes are available. Please follow the instruction on their [website](http://gazebosim.org/wiki/Main_Page) for installation.

**Simulator Repositories**

Development of the SMORES simulator is currently split into two repositories: one for [plugins](https://github.com/MOD-ASL/SimulationPlugins/) and one for [models](https://github.com/MOD-ASL/GAZEBO_model). The plugins repository contains the code used to control the SMORES modules and generate custom behaviors in the simulator, such as the docking and undocking actions between two modules. The models repository contains the description of the robot models, sensor models, and world parameters. This repository uses the plugins created in the plugin repository to give the models their functionality.

To get the simulator, clone the two repositories using git. In order to get the sensor models and testing platform, you will additionally need to fetch the sensors branch on the amptrofa/ fork of each of the repositories. As of this writing, the models repository contains compiled binary files of the plugins; you may need to recompile these for your platform.

**Simulation Setup**

The instruction of setting up the simulation could be found on the github wiki of this project through this [link](https://github.com/MOD-ASL/SimulationPlugins/wiki/Simulation-Setup).

**File Relationships**

In order to use the models from the models repository in Gazebo, they must be copied to the ~/.gazebo/models directory. The simplest thing to do is copy all directories in the GAZEBO_models repository to this location.
In order to use a plugin within a model, Gazebo needs to know where to find the plugin. Currently, all of the plugins used by a given model are in the ~.gazebo/models/&lt;model name&gt;/plugins/ directory, and are referenced with a relative path as "plugins/&lt;plugin&gt;.so". Gazebo is then invoked with either the commands "gzserver" or "gazebo" from the &lt;model name&gt;/ directory, and finds the plugins relative to this directory. As such, even plugins used by models that are attached to the current model via an &lt;include&gt; (for example, the sensor models) must be in the &lt;model name&gt;/plugins directory for any model that they will be used with.

An alternative to the above steps is to export the paths of the models and plugins to the Gazebo model path and Gazebo plugin path. This will likely be the preferred option once the simulator has reached a more stable state of development. A short tutorial could be found [here](http://gazebosim.org/wiki/Tutorials/1.9/plugins/overview).

**Using the Simulator**

The most up to date version of the SMORES simulator is currently in the SMORES6Uriah model. To run the simulator, navigate to this model's folder and run "gazebo World.sdf -u". This will start the simulator with parameters described in "World.sdf" and paused (-u). The initial configuration will be loaded into the simulation world if specified, and commands will be executed if there is any.

A short video could be found [here](https://www.youtube.com/watch?v=eX9czliCpnM)

We also designed Configuration designer tools and command recorder for people who want to play with this simulator. The instruction of how to setup and use these tools can also be found on this project github [wiki](https://github.com/MOD-ASL/SimulationPlugins/wiki).

People from both lab are preparing a competetion on the configration design and gait control design, the competetion kit will also be launched soon.

**For developers**

You may find the APIs(documentation) for all the simulation plugins and python program on this [page](http://mod-asl.github.io/SimulationPlugins/apis/).

Known problems of GAZEBO
----------------------------------------
1. In the world plugin, when add entity event called response function, the entity itself(The object) hasn't been created, which means we cannot get the pointer pointing to that object. In our case that entity is a model. So the temporary solution is to set the pointer somewhere else. Haven't searched for gazebo answer or post a question. Will do it later.

MEMO
----------------------------------------
Note file has been moved to this [link](https://github.com/MOD-ASL/SimulationPlugins/wiki/Development-Note) on the github wiki.

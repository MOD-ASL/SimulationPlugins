S.M.O.R.E.S. Simulator
=================

Author: Yunkai (Edward) Cui, Modlab, University of Pennsylvania<br>
Co-author: Adam Trofa, Cornell University<br>
Co-author: Gangyuan (Jim) Jing, Cornell University

Started Date: 08/2013

Last Edti: 12/24/2013 


Introduction
----------------------------------------
The SMORES simulator aims to model the capabilities of the SMORES modular robot platform, allowing researchers to rapidly assess the capabilities of different hardware configurations and develop the algorithms required to exercise the powerful flexibility available to them. In addition to providing accurate dynamics modeling of a given robotic configuration, the simulator aims to model the reconfiguration and sensing capabilities of the SMORES modules.

The core physics and control of the simulator are currently being developed at Modlab at Penn using the simulation software Gazebo. The sensor add-ons that can be used to provide individual modules with specific sensing capabilities are being developed by Autonomous Systems Lab at Cornell. Both groups are cooperating on a NSF funded project.

For More information about the Gazebo simulation engine, please visit [GAZEBOsim](http://gazebosim.org/)

For more information about Modlab at Penn, please visit [Modlab](http://modlabupenn.org/)

For more information about Autonomous Systems Lab at Cornell, please visit [Autonomous Systems Lab](http://cornell-asl.org/wiki/index.php?title=Main_Page)

Installation and Quick Start
----------------------------
**Gazebo**

To use the simulator, you will first need to install Gazebo. Gazebo may be compiled from source, but if you are using Ubuntu 12.04, 12.10, 13.04, or 13.10, precompiled binaries are available to install via apt-get. However, the precompiled version may be compiled with a different version of protoc. If that is the case, one has to recompile gazebo from source file. Add the appropriate repository for your distribution and install according to the instructions in the [Gazebo Wiki](http://gazebosim.org/wiki/1.9/install#Pre-compiled_binaries). It is highly recommended to use GAZEBO 1.9, because GAZEBO 2.0 or 2.1 is currently not a full package.

Notes for installation from source file: the instruction on the Gazebo wiki is very clear and easy to follow. Just make sure to check the ~/.bashrc file after installation to make sure the path is properly set.

**Simulator Repositories**

Development of the SMORES simulator is currently split into two repositories: one for [plugins](https://github.com/princeedward/SimulationPlugins/) and one for [models](https://github.com/princeedward/GAZEBO_model). The plugins repository contains the code used to control the SMORES modules and generate custom behaviors in the simulator, such as the docking and undocking actions between two modules. The models repository contains the description of the robot models, sensor models, and world parameters. This repository uses the plugins created in the plugin repository to give the models their functionality.

To get the simulator, clone the two repositories using git. In order to get the sensor models and testing platform, you will additionally need to fetch the sensors branch on the amptrofa/ fork of each of the repositories. As of this writing, the models repository contains compiled binary files of the plugins; you may need to recompile these for your platform.

**Compiling Plugins**

***Note:*** <br>
1. World plugin is in the WorldPlugin/ folder <br>
2. Model plugin is in the ModelPlugin/ folder <br>
3. In the ContactSensor/ folder there is an important library used for simulating the magnetic connection

Each plugin has its own folder in the plugins repository. To compile a plugin, create a build/ directory within that plugin's folder. From the new directory, run the commands:<br>
 <code>cmake ../</code><br>
 <code>make</code>

This should compile the plugin build/lib<pluginname>.so. You should copy this file into the appropriate folder for your model.

**Note**: The Sensor_Testbed model used to test the sensors should probably be run with the libControllerPro1.so file compiled from amptrofa/SimulationPlugins/sensors, which is different from the plugin on other branches. This is a stable plugin used for testing.

**File Relationships**

In order to use the models from the models repository in Gazebo, they must be copied to the ~/.gazebo/models directory. The simplest thing to do is copy all directories in the GAZEBO_models repository to this location.
In order to use a plugin within a model, Gazebo needs to know where to find the plugin. Currently, all of the plugins used by a given model are in the ~.gazebo/models/<model name>/plugins/ directory, and are referenced with a relative path as "plugins/<plugin>.so". Gazebo is then invoked with either the commands "gzserver" or "gazebo" from the <model name>/ directory, and finds the plugins relative to this directory. As such, even plugins used by models that are attached to the current model via an <include> (for example, the sensor models) must be in the <model name>/plugins directory for any model that they will be used with.

An alternative to the above steps is to export the paths of the models and plugins to the Gazebo model path and Gazebo plugin path. This will likely be the preferred option once the simulator has reached a more stable state of development. A short tutorial could be found [here](http://gazebosim.org/wiki/Tutorials/1.9/plugins/overview).

**Using the Simulator**

The most up to date version of the SMORES simulator is currently in the SMORES5Jon model. To run the simulator, navigate to this model's folder and run "gazebo World.sdf -u". This will start the simulator with parameters described in "World.sdf" and paused (-u). You can now select the SMORES5Jon model and insert it from the models menu in the Gazebo GUI. Running the simulation, any modules you place in the scene will drive towards the point (1,1), and they will dock together once they are close to each other.

A short video could be found [here](https://www.youtube.com/watch?v=U9dJSMIzxhA)

The Sensor_Testbed model contains the code to include each of the sensors. To run the sensor testbed and view the sensor output, navigate to the model's folder and run "gazebo -u". This is the same as above, only no world description is required. Currently code to include all of the sensors is in the SMORE.sdf description file, but commented out. Uncomment the inclusion code for any of the sensors you wish to test to use these sensors. Any models added in Gazebo after changes are made to the SMORE.sdf file will include those changes.

Known problems of GAZEBO
----------------------------------------
1. It doesn't provide a function to delete an entity, this feature probabily wull be supported in the future, which also means we likely cannot delete the dynamic joints cleanly. For more information, please see the link [here](http://answers.gazebosim.org/question/550/how-to-delete-links-from-a-model-gazebo-125/).
2. In the world plugin, when add entity event called response function, the entity itself(The object) hasn't been created, ehich means we cannot get the pointer pointing to that object. In our case that entity is a model. So the temporary solution is to set the pointer somewhere else. Haven't searched for gazebo answer or post a question. Will do it later.
3. One method from boost library GAZEBO 1.9 using now is no longer supported by the newest version of boost library. The known version of boost library that compatible with GAZEBO 1.9 is boost 1.46

MEMO
----------------------------------------
1. shared_ptr: make_shared function will not generate a pointer point to the input variable. Instead, this function will copy the input variable to a new address and makes a pointer point to the new address.
2. For c++ compiler, it is OK that you define a method(function) in the class in the header file, but you don't specify it in the .cc file. However it is not ok in gazebo. It will show a symbol undefined error. 

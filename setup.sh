#!/bin/bash
# To be able to run this file, you have to make sure you have the following dependencies installed
. /etc/lsb-release

STATUS=1
WIPEOUT=0
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ $# -eq 1 ]; then
  if [ "$1" = "-s" ]; then
    STATUS=2
    if [ $DISTRIB_RELEASE = "12.04" ]; then
      sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'
    elif [ $DISTRIB_RELEASE = "12.10" ]; then
      sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu quantal main" > /etc/apt/sources.list.d/gazebo-latest.list'
    elif [ $DISTRIB_RELEASE = "13.04" ]; then
      sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu raring main" > /etc/apt/sources.list.d/gazebo-latest.list'
    elif [ $DISTRIB_RELEASE = "13.10" ]; then
      sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu saucy main" > /etc/apt/sources.list.d/gazebo-latest.list'
    else
      echo "[WARNING] Cannot execute automatically setup for this OS distribution"
      STATUS=0
    fi
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
    apt-get update
    if [ $DISTRIB_RELEASE = "12.04" ]; then
      apt-get install gazebo-current
    elif [ $DISTRIB_RELEASE = "12.10" ]; then
      apt-get install gazebo-current
    elif [ $DISTRIB_RELEASE = "13.04" ]; then
      apt-get install gazebo-current
    elif [ $DISTRIB_RELEASE = "13.10" ]; then
      apt-get install gazebo2
    else
      echo "[WARNING] Mission abort"
      STATUS=0
    fi
    if [ $STATUS -ne 0 ]; then
      echo "[Installing] python ..."
      apt-get install python
      echo "[Installing] gcc & g++ ..."
      apt-get install build-essential
      echo "[Installing] cmake ..."
      apt-get install cmake
      echo "[Installing] extra python packages ..."
      apt-get install python-pip
      pip install -U pip
      pip install eventlet
      pip install pygazebo
      apt-get install python-imaging-tk
      apt-get install python-numpy python-scipy
      echo "[Installing] boost library ..."
      apt-get install libboost-all-dev
      echo "[Installing] google protobuf compiler ..."
      apt-get install protobuf-compiler
    fi
  fi
  if [ "$1" = "-u" ]; then
    STATUS=2
    WIPEOUT=1
  fi
  if [ "$1" = "-w" ]; then
    WIPEOUT=1
  fi
elif [ $# -ge 2 ]; then
  echo "[WARNING] Script cannot take more than one arguments"
  STATUS=0
fi

if [ "$STATUS" = 2 ]; then
  echo "Pulling models from github repo ..."
  cd ~/.gazebo/models
  git init
  git add .
  git commit -m "Model backup"
  git pull git@github.com:princeedward/GAZEBO_model.git master
  STATUS=1
  cd "$DIR"
  git add .
  git commit -m "Backup before update"
  git pull git@github.com:princeedward/SimulationPlugins.git master
fi

if [ "$STATUS" = 1 ]; then
  echo "Update all necessary plugins ..."
  cd "$DIR"
  cd WorldController/
  if [ ! -d build/ ]; then
    mkdir build
    cd build
    cmake ../
  else
    cd build
    if [ "$WIPEOUT" = 1 ]; then
      rm -r *
      cmake ../
    fi
  fi
  make
  cp *.so ~/.gazebo/models/SMORES6Uriah/plugins/
  cp MessageDefinition/*.so ~/.gazebo/models/SMORES6Uriah/plugins/MessageDefinition/

  cd "$DIR"
  cd ModuleController/
  if [ ! -d build/ ]; then
    mkdir build
    cd build
    cmake ../
  else
    cd build
    if [ "$WIPEOUT" = 1 ]; then
      rm -r *
      cmake ../
    fi
  fi
  make
  cp *.so ~/.gazebo/models/SMORES6Uriah/plugins/
  cp *.so ~/.gazebo/models/SMORES7Stella/plugins/
  cp *.so ~/.gazebo/models/SMORES8Jack/plugins/

  cd "$DIR"
  cd ContactPublisher/
  if [ ! -d build/ ]; then
    mkdir build
    cd build
    cmake ../
  else
    cd build
    if [ "$WIPEOUT" = 1 ]; then
      rm -r *
      cmake ../
    fi
  fi
  make
  cp *.so ~/.gazebo/models/SMORES6Uriah/plugins/
  cp *.so ~/.gazebo/models/SMORES7Stella/plugins/
  cp *.so ~/.gazebo/models/SMORES8Jack/plugins/

  cd "$DIR"
  cd ConfigGenerator/pythonGUI
  sh updatemessageproto.sh
  cd ../guiplugin/
  if [ ! -d build/ ]; then
    mkdir build
    cd build
    cmake ../
  else
    cd build
    if [ "$WIPEOUT" = 1 ]; then
      rm -r *
      cmake ../
    fi
  fi
  make
  export GAZEBO_PLUGIN_PATH="$PWD":"$GAZEBO_PLUGIN_PATH"
  cd ../../worldplugin/
  if [ ! -d build/ ]; then
    mkdir build
    cd build
    cmake ../
  else
    cd build
    if [ "$WIPEOUT" = 1 ]; then
      rm -r *
      cmake ../
    fi
  fi
  make
  cp *.so ~/.gazebo/models/SMORES7Stella/plugins/
  cp MessageDefinition/*.so ~/.gazebo/models/SMORES7Stella/plugins/MessageDefinition/

  cd "$DIR"
  cd GaitRecorder/pythonGUI
  sh setup.sh
  cd ../worldplugin/
  if [ ! -d build/ ]; then
    mkdir build
    cd build
    cmake ../
  else
    cd build
    if [ "$WIPEOUT" = 1 ]; then
      rm -r *
      cmake ../
    fi
  fi
  make
  cp *.so ~/.gazebo/models/SMORES8Jack/plugins/
  cp MessageDefinition/*.so ~/.gazebo/models/SMORES8Jack/plugins/MessageDefinition/
  echo "Setup finished!"
fi
cd "$DIR"
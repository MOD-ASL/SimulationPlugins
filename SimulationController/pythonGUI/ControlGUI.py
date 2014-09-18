#!/usr/bin/python

import utils.rpErrorHandler as rpErrorHandler
from Tkinter import *
#------------------------------------------------------------------------------#
#                                                                              #
#                                  ControlGUI                                  #
#                                                                              #
#------------------------------------------------------------------------------#
class ControlGUI(Frame):
    def __init__(self,Master=None,**kw):
        #
        #Your code here
        #

        apply(Frame.__init__,(self,Master),kw)
        self.__Frame9 = Frame(self,width=30)
        self.__Frame9.pack(side='left')
        self.__Frame10 = Frame(self)
        self.__Frame10.pack(side='left')
        self.__Frame3 = Frame(self)
        self.__Frame3.pack(side='left')
        self.__Frame5 = Frame(self.__Frame10)
        self.__Frame5.pack(side='top')
        self.__Listbox_configuration = Listbox(self.__Frame5,height=20,width=35)
        self.__Listbox_configuration.pack(fill='both',side='top')
        self.__Frame1 = Frame(self.__Frame10)
        self.__Frame1.pack(side='top')
        self.__Button_importConfig = Button(self.__Frame1
            ,text='Import Configuration')
        self.__Button_importConfig.pack(side='top')
        self.__Frame6 = Frame(self.__Frame10)
        self.__Frame6.pack(side='top')
        self.__Button_loadWorld = Button(self.__Frame6,text='Load World')
        self.__Button_loadWorld.pack(side='left')
        self.__Button_loadLibrary = Button(self.__Frame6,text='Load Library')
        self.__Button_loadLibrary.pack(side='right')
        self.__Frame4 = Frame(self.__Frame3)
        self.__Frame4.pack(side='top')
        self.__Frame11 = Frame(self.__Frame3)
        self.__Frame11.pack(side='top')
        self.__Frame2 = Frame(self.__Frame3)
        self.__Frame2.pack(side='top')
        self.__Frame7 = Frame(self.__Frame4,width=10)
        self.__Frame7.pack(side='left')
        self.__Label_scoreText = Label(self.__Frame7,text='Score')
        self.__Label_scoreText.pack(side='top')
        self.__Label_scoreNum = Label(self.__Frame7,width=20)
        self.__Label_scoreNum.pack(side='top')
        self.__Frame8 = Frame(self.__Frame4)
        self.__Frame8.pack(side='left')
        self.__Listbox_gait = Listbox(self.__Frame8,height=20,width=35)
        self.__Listbox_gait.pack(fill='both',side='top')
        self.__Frame12 = Frame(self.__Frame11)
        self.__Frame12.pack(ipadx=20,padx=60,side='left')
        self.__Frame13 = Frame(self.__Frame11)
        self.__Frame13.pack(side='left')
        self.__Button_executeGait = Button(self.__Frame13,text='Execute Gait')
        self.__Button_executeGait.pack(side='top')
        self.__Frame15 = Frame(self.__Frame2)
        self.__Frame15.pack(ipadx=20,padx=60,side='left')
        self.__Frame14 = Frame(self.__Frame2)
        self.__Frame14.pack(side='left')
        self.__Button_close = Button(self.__Frame14,text='Close')
        self.__Button_close.pack(side='top')

        #
        #Your code here
        #

        # initialize variables
        self.smores_library = [] # a list of smores library entries
        self.communicator = None # the communicator to talk with gazebo

        # event binding
        self.__Listbox_configuration.bind('<<ListboxSelect>>', self.onSelectConfig)
        self.__Button_importConfig["command"] = self.onClickImportConfig
        self.__Button_executeGait["command"] = self.onClickExecuteGait
        self.__Button_loadWorld["command"] = self.onClickLoadWorld
        self.__Button_loadLibrary["command"] = self.onClickLoadLibrary
        self.__Button_close["command"] = self.onClickClose

        # disable some buttons initially
        self.__Button_importConfig["state"] = DISABLED
        self.__Button_loadLibrary["state"] = DISABLED
        self.__Button_executeGait["state"] = DISABLED
    #
    #Start of event handler methods
    #
    def onClose(self):
        """
        Event handler for when the GUI is closed
        """
        if self.communicator:
            self.communicator.Close()
            self.rungazebo.terminate()
            call(["pkill", "gzserver"])
            call(["pkill", "gzclient"])

    def onSelectConfig(self, event):
        """
        Event handler for when a configuration is selected
        in the configuration list box
        """

        # get the configuration file name selected
        w =  event.widget
        index = int(w.curselection()[0])
        value = w.get(index)

        # populate the gait list for the selected configuration
        self.populateGaitList(value)

    def onClickImportConfig(self):
        """
        Event handler for when click the import configuration button
        Send the current selected configuration file name to gazebo
        """
        # get the configuration file name selected
        w = self.__Listbox_configuration
        index = int(w.curselection()[0])
        value = w.get(index)

        # send the file name
        self.sendConfiguration(value)

    def onClickExecuteGait(self):
        """
        Event handler for when click the execute gait button
        Send the current selected gait file name to gazebo
        """
        # get the configuration file name selected
        w = self.__Listbox_gait
        index = int(w.curselection()[0])
        value = w.get(index)

        # send the file name
        self.sendGait(value)

    def onClickLoadWorld(self):
        """
        Event handler for when click the load world button
        Run gazebo with selected world file
        """
        world_file_name = ''
        options = {}
        options['filetypes'] = [('sdf files', '.sdf')]
        options['initialdir'] = '~/.gazebo/models/SMORES8Jack/'
        options['title'] = 'Open World File'
        world_file_name = tkFileDialog.askopenfilename(**options)
        if world_file_name == '':
            return

        # disable the load world button
        self.__Button_loadWorld["state"] = DISABLED
        self.__Button_loadLibrary["state"] = NORMAL

        # initialize
        self.initialize(os.path.basename(world_file_name))

    def onClickLoadLibrary(self):
        """
        Event handler for when click the load library button
        Set the SMORES configuration and gait library path
        """

        smores_library_path = ''
        options = {}
        options['initialdir'] = '~/Research/SMORES/SimulationPlugins/SimulationController/pythonGUI/'
        options['title'] = 'Open SMORES Library'
        smores_library_path = tkFileDialog.askdirectory(**options)
        if smores_library_path == '':
            return

        self.loadSMORESLibrary(smores_library_path)
        self.populateConfigList()
        self.__Button_loadLibrary["state"] = DISABLED
        self.__Button_importConfig["state"] = NORMAL
        self.__Button_executeGait["state"] = NORMAL

        # tell gazebo where to load the smores library
        self.writeLibraryPath(smores_library_path)

    def onClickClose(self):
        """
        Event handler for when click the close button
        Close and terminate everything
        """
        self.onClose()
        self.quit()

    def populateConfigList(self):
        """
        Populate the list box for configuration
        """
        for sle_object in self.smores_library:
            self.__Listbox_configuration.insert(END, sle_object.configuration_file_name)

        # by default select the first item
        self.__Listbox_configuration.selection_set(0)
        self.populateGaitList(self.__Listbox_configuration.get(0))

    def populateGaitList(self, configuration_file_name):
        """
        Populate the list box for gait of `configuration_file_name`
        """

        # delete the existing list first
        self.__Listbox_gait.delete(0, END)

        # populate the list
        for sle_object in self.smores_library:
            if sle_object.configuration_file_name == configuration_file_name:
                for gait_file in sle_object.gait_file_name_list:
                    self.__Listbox_gait.insert(END, gait_file)
                break

    #
    #Start of non-Rapyd user code
    #

    def initialize(self, world_file_name):
        """
        Initialize everything we need
        """
        self.rungazebo = Popen(['sh', 'RunSimulation.sh', '-w', world_file_name])
        time.sleep(2)
        self.communicator = gztopic.GzCommunicator() # initialize and start the communicator
        self.communicator.start()
        self.sim_control_publisher = self.communicator.CreatePulisher("/gazebo/SimulationController/simControlSubscriber",'sim_control_message.msgs.SimControlMessage')

    def loadSMORESLibrary(self, smores_library_path):
        """
        Load the SMORES library
        """
        # walk down the library to load all data
        for root, dirs, files in os.walk(smores_library_path):
            # ignore the top most directory
            if root == smores_library_path: continue

            # otherwise load the config and gait data
            # each configuration is stored in a folder which includes
            # one .config file and multipule .gait files
            for file in files:
                if os.path.splitext(file)[1] == '.conf':
                    # this directory has a .config file
                    # this is a valid configuration entry
                    sle_object = SLE.SmoresLibraryEntry()
                    sle_object.entry_dir_path = root

                    # store the configuration file name
                    sle_object.configuration_file_name = file

                    # store the gait files if there is any
                    sle_object.gait_file_name_list = [f for f in files \
                            if os.path.splitext(f)[1] == '.gait']

                    # store the SMORES library entry
                    self.smores_library.append(sle_object)
                    break

    def sendConfiguration(self, configuration_file_name):
        """
        Tell gazebo to load the selected configuration
        """
        msg = sc_message.SimControlMessage()
        msg.ConfigurationName = configuration_file_name
        self.sim_control_publisher.Publish(msg)

    def sendGait(self, gait_file_name):
        """
        Tell gazebo to load the selected gait
        """
        msg = sc_message.SimControlMessage()
        msg.GaitName = gait_file_name
        msg.ConfigurationName = "" # since this field is required
        self.sim_control_publisher.Publish(msg)

    def writeLibraryPath(self, path):
        """
        Write the location of SMORES library directory to ~/.gazebo/models/SMORES8Jack/SMORES_LIBRARY_PATH_FILE
        """
        home = expanduser("~")
        f = open(os.path.join(home, '.gazebo/models/SMORES8Jack/SMORES_LIBRARY_PATH_FILE'), 'w')
        f.write(path)
        f.close()

try:
    #--------------------------------------------------------------------------#
    # User code should go after this comment so it is inside the "try".        #
    #     This allows rpErrorHandler to gain control on an error so it         #
    #     can properly display a Rapyd-aware error message.                    #
    #--------------------------------------------------------------------------#

    #Adjust sys.path so we can find other modules of this project
    import sys
    if '.' not in sys.path:
        sys.path.append('.')
    sys.path.append("../../Util/python_util")
    import gztopic_multithreading as gztopic
    import os
    from os.path import expanduser
    import time
    import tkFileDialog
    from subprocess import Popen, PIPE, call
    import SmoresLibraryEntry as SLE
    import sim_control_message_pb2 as sc_message

    if __name__ == '__main__':

        Root = Tk()
        import Tkinter
        Tkinter.CallWrapper = rpErrorHandler.CallWrapper
        del Tkinter
        App = ControlGUI(Root)
        App.pack(expand='yes',fill='both')

        Root.geometry('800x480+10+10')
        Root.title('ControlGUI')
        Root.mainloop()
    #--------------------------------------------------------------------------#
    # User code should go above this comment.                                  #
    #--------------------------------------------------------------------------#
except:
    rpErrorHandler.RunError()
finally:
    App.onClose()

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
        self.__Frame4 = Frame(self.__Frame3)
        self.__Frame4.pack(side='top')
        self.__Frame2 = Frame(self.__Frame3)
        self.__Frame2.pack(side='top')
        self.__Button_executeGait = Button(self.__Frame2,text='Execute Gait')
        self.__Button_executeGait.pack(side='left')
        self.__Button_stopGait = Button(self.__Frame2,text='Stop Gait')
        self.__Button_stopGait.pack(side='right')
        self.__Frame7 = Frame(self.__Frame4,width=10)
        self.__Frame7.pack(side='left')
        self.__Frame8 = Frame(self.__Frame4)
        self.__Frame8.pack(side='left')
        self.__Listbox_gait = Listbox(self.__Frame8,height=20,width=35)
        self.__Listbox_gait.pack(fill='both',side='top')
        #
        #Your code here
        #

        # initialize variables
        self.smores_library = [] # a list of smores library entries
        self.smores_library_path = 'SmoresLibrary' # the directory of the SMORES library
        self.communicator = None # the communicator to talk with gazebo

        # event binding
        self.__Listbox_configuration.bind('<<ListboxSelect>>', self.onSelectConfig)
        self.__Button_importConfig.bind('<Button-1>', self.onClickImportConfig)
        self.__Button_executeGait.bind('<Button-1>', self.onClickExecuteGait)

        # initialize
        self.initialize()


    #
    #Start of event handler methods
    #
    def onClose(self):
        """
        Event handler for when the GUI is closed
        """
        self.communicator.Close()
        self.rungzserver.terminate()
        self.rungzclient.terminate()
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

    def onClickImportConfig(self, event):
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

    def onClickExecuteGait(self, event):
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

    def initialize(self):
        """
        Initialize everything we need
        """
        self.loadSMORESLibrary()
        self.populateConfigList()
        self.rungzserver = Popen(['sh', 'RunSimulation.sh'])
        self.rungzclient = Popen(['gzclient'], stdout=PIPE)
        time.sleep(2)
        self.communicator = gztopic.GzCommunicator() # initialize and start the communicator
        self.communicator.start()
        self.sim_control_publisher = self.communicator.CreatePulisher("/gazebo/SimulationController/simControlSubscriber",'sim_control_message.msgs.SimControlMessage')

    def loadSMORESLibrary(self):
        """
        Load the SMORES library
        """
        # walk down the library to load all data
        for root, dirs, files in os.walk(self.smores_library_path):
            # ignore the top most directory
            if root == self.smores_library_path: continue

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
    import time
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

        Root.geometry('640x480+10+10')
        Root.title('ControlGUI')
        Root.mainloop()
    #--------------------------------------------------------------------------#
    # User code should go above this comment.                                  #
    #--------------------------------------------------------------------------#
except:
    rpErrorHandler.RunError()
finally:
    App.onClose()

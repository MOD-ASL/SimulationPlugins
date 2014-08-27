## @package GaitRecorder Gait recorder python GUI application

#--------------- System related ----------------------
import time
from subprocess import call, Popen, PIPE
import sys
#--------------- GUI Modules -------------------------
from Tkinter import *
import ttk
import tkFileDialog
from PIL import Image, ImageTk  # need to install: sudo apt-get install python-imaging-tk
#--------------- Representation related --------------
import xml.etree.ElementTree as ET   # XML parser
from Module import *
from Connection import *
from AssociateJoint import *
from GaitEntry import *
from Section import *
#--------------- Communiation related ----------------
sys.path.append("../../Util/python_util")
from gait_recorder_message_pb2 import *
# import eventlet  # need to install: $:sudo pip install eventlet
# from pygazebo import *  #need to install: $: sudo pip install pygazebo
from gztopic import *
#--------------- Mathematic related ------------------
import numpy as np
#--------------- Debuggin Tools ----------------------
import pdb
#------------- Window Size Settings ------------------
## Window width
window_width = 800
## Window height
window_height = 520
## Window left and right padding
Border_width = 20
## Window up and bottom padding
Border_hieht = 40
## Dialog window width
DIALOG_WIDTH = 400
## Dialog window height
DIALOG_HEIGHT = 260
## PI used in this module
PI = np.pi
## This is the class of gait recorder  python gui application
class GaitRecorder(Frame):
  ## Constructor
  # @param self Object pointer
  # @param parent Parent of this app, which is tk root
  # @param flag Specifies the state of the this python program, 
  # 0 for normal mode, 1 for gui debug mode, 2 for python only mode
  def __init__(self, parent, flag):
    Frame.__init__(self, parent)   
    #------------ Variables Initialization --------------- 
    ## Parent of the App class, which is a tk root
    self.parent = parent
    ## Variable stores name string in the wedget
    self.modelname = StringVar()

    #----------- Common Command Entry Section ------------
    self.jointSelection = IntVar()
    self.commandType = IntVar()
    self.typeSelection = IntVar()
    self.associatedJointsList = StringVar()
    self.nodeSelect1 = IntVar()
    self.nodeSelect2 = IntVar()
    self.module2Select = StringVar()
    self.jointAngleDifferenceTracking = [0]*4
    ## Variable records the entered value right next to scroll bar
    self.valueInBox = DoubleVar()
    #----------- Extra Information Section --------------
    self.condition = StringVar()
    self.dependency = StringVar()
    self.elapstime = DoubleVar()
    self.elapsedTimeType = StringVar()
    #----------- Save Path Section -----------------------
    self.savepath = StringVar()
    #----------- Manually Update Section ----------------- 
    self.selectedcommand = StringVar()
    #------------ Command Recorder -----------------------
    self.selectedkeyFrame = StringVar()
    #------------ Save Path Selection --------------------
    self.savePath = "~/"
    #------------ Joint Associated Joints ----------------
    self.frontWheelAssociates = []
    self.lftWheelAssociates = []
    self.rgtWheelAssociates = []
    self.centralBendAssociates = []
    self.currentAssociates = []
    #------------ Gait History Related ------------------
    self.commandhis = StringVar()
    self.frameListForGaitRecord =[]
    self.frameButtonList = []
    self.frameListList = []
    self.keyFrames = []
    self.keyFrameList = []

    self.modulenames = []
    # self.FrameList = []
    self.CurrentFrameRec = []
    self.CurrentFrameHis = []
    self.DependencyList = []
    self.ModuleList = []
    self.ConnectionList = []
    self.ConnectableModules = []
    self.DisconnectableModules = []
    self.initflag = flag
    #-------------- File Definition ---------------------------
    # define options for opening or saving a file
    self.file_opt = options = {}
    # options['defaultextension'] = '.txt'
    options['filetypes'] = [('all files', '*'), ('text files', '.txt')]
    options['initialdir'] = '~/'
    # options['initialfile'] = 'myfile.txt'
    options['parent'] = parent
    options['title'] = 'Open Configuration File'

    #------------ Run Simulation and GUI ---------------------
    if flag == 0: 
      self.rungzserver = Popen(['sh', 'RunSimulation.sh'], stdout=PIPE)
      time.sleep(1)
      # self.rungzclient = Popen(['gzclient'], stdout=PIPE)
      # time.sleep(1)

    #-------------- Establish Connection With Simulator -------
    if flag == 0 or flag == 2:
      self.communicator = GzCommunicator()
      self.communicator.StartCommunicator("/gazebo/GaitRecorder/gaitSubscriber","gait_recorder_message.GaitRecMessage")

    self.initUI()
    # self.SaveCurrentPose()
    self.SelectCommonCommand()
  ## Initializes the UI of the current python app
  # @param self Object pointer    
  def initUI(self):
    self.parent.title("Gait Table Recorder")
      
    self.pack(fill=BOTH, expand=1)
    # okButton = Button(self, text="OK")
    # okButton.pack(side=RIGHT)
    n = ttk.Notebook(self)
    f1 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED); # first page, which would get widgets gridded into it
    f3 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED)
    f2 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED); # second page
    n.add(f1, text='Record New Gaits')
    n.add(f3, text='Gait Records')
    n.add(f2, text='Manage Gait Table')
    n.pack()

    # --------------- Close Button ------------------------------
    closeButton = Button(f1, text="Close")
    closeButton["command"] = self.CloseWindow
    closeButton.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)
    closeButton2 = Button(f2, text="Close")
    closeButton2["command"] = self.CloseWindow
    closeButton2.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Model Name ---------------------------------
    label = Label(f1, text='Select Model: ')
    label.place(x = 10, y = 10)
    self.name = ttk.Combobox(f1, textvariable=self.modelname, width = 10) #, command = self.checkConnectivity
    self.name['values'] = ()
    self.name.bind('<<ComboboxSelected>>',self.UpdateJoint)
    self.name.place(x = 100, y = 10)

    #--------------- Joint Angle Modification -------------------
    JointModSec = ttk.Labelframe(f1, text='Common Command Entry ', width = 760, height = 250)
    JointModSec.place(x = 10, y = 40)

    self.selectCommonCommand = Radiobutton(JointModSec, text='Add Joint Update Command', variable=self.commandType, value=0, command = self.SelectCommonCommand)
    self.selectCommonCommand.place(x = 10, y = 0)

    label2 = Label(JointModSec, text='1. Select Joint ')
    label2.place(x = 10, y = 25)
    bard = Image.open("SmallSmores.png")
    bardejov = ImageTk.PhotoImage(bard)
    label3 = Label(JointModSec, image=bardejov)
    label3.image = bardejov
    label3.place(x=90, y=70)
    self.bend_joint = Radiobutton(JointModSec, text='Central Bending', variable=self.jointSelection, value=3, command = self.UpdateJointValue)
    self.left_joint = Radiobutton(JointModSec, text='Lft Wheel', variable=self.jointSelection, value=1, command = self.UpdateJointValue)
    self.right_joint = Radiobutton(JointModSec, text='Rgt Wheel', variable=self.jointSelection, value=2, command = self.UpdateJointValue)
    self.front_joint = Radiobutton(JointModSec, text='Front Wheel', variable=self.jointSelection, value=0, command = self.UpdateJointValue)
    self.front_joint.select()
    self.bend_joint.place(x= 125, y = 55,anchor = CENTER)
    self.front_joint.place(x= 125, y = 165,anchor = CENTER)
    self.right_joint.place(x= 45, y = 120,anchor = CENTER)
    self.left_joint.place(x= 215, y = 120,anchor = CENTER)

    label4 = Label(JointModSec, text='2. Select Type ')
    label4.place(x = 280, y = 25)
    self.modeAngle = Radiobutton(JointModSec, text='Angle', variable=self.typeSelection, value=0, command = self.UpdateJointValue)
    self.modeSpeed = Radiobutton(JointModSec, text='Speed', variable=self.typeSelection, value=1, state=DISABLED, command = self.UpdateJointValue)
    self.modeTorque = Radiobutton(JointModSec, text='Torque', variable=self.typeSelection, value=2, state=DISABLED, command = self.UpdateJointValue)
    self.modeAngle.select()
    self.modeAngle.place(x = 280, y = 60)
    self.modeSpeed.place(x = 280, y = 90)
    self.modeTorque.place(x = 280, y = 120)
    label5 = Label(JointModSec, text='4. Select Value ')
    label5.place(x = 10, y = 175)
    self.valueSetting = Scale(JointModSec, from_=-180, to=180, orient=HORIZONTAL,length = 320, resolution = 5, state = NORMAL, command = self.DynamicUpdate)
    self.valueSetting.place(x = 10, y = 190)
    self.valueSettingBox = Entry(JointModSec, width=6, textvariable=self.valueInBox)
    self.valueSettingBox.bind('<Return>',self.UpdateFromValueBox)
    self.valueSettingBox.place(x = 340, y = 205)

    label6 = Label(JointModSec, text='3. Select Associated Joints ')
    label6.place(x = 400, y = 0)
    self.associatedJoints = Listbox(JointModSec, width=40, height=10,listvariable = self.associatedJointsList)
    self.associatedJoints.place(x = 400, y = 60)
    joint_scroller = Scrollbar(JointModSec, command=self.associatedJoints.yview)
    self.associatedJoints.config(yscrollcommand=joint_scroller.set)
    joint_scroller.place(x = 723, y = 60, height = 165)
    self.ascModify = Button(JointModSec, text='Add', width = 8, command = self.AddAssociates)
    self.ascModify.place(x = 400, y = 30)
    self.ascDelete = Button(JointModSec, text='Delete', width = 8, command = self.DeleteAnAssociate)
    self.ascDelete.place(x = 645, y = 30)

    #---------------- Connection & Disconnection ----------------
    SpecialCommandSec = ttk.Labelframe(f1, text='Sepcial Command Entry ', width = 350, height = 140)
    SpecialCommandSec.place(x = 10, y = 300)
    label_sc1 = Label(SpecialCommandSec, text = "Add")
    label_sc1.place(x = 10,y = 0)
    self.connectionCommand = Radiobutton(SpecialCommandSec, text='Connection', variable=self.commandType, value=1, command = self.SelectConnection)
    self.connectionCommand.place(x = 40, y = 0)
    label_sc2 = Label(SpecialCommandSec, text = "or")
    label_sc2.place(x = 150, y = 0)
    self.disconnectionCommand = Radiobutton(SpecialCommandSec, text='Disconnection', variable=self.commandType, value=2, command = self.SelectDisconnection)
    self.disconnectionCommand.place(x = 170, y = 0)
    label_sc3 = Label(SpecialCommandSec, text = "Node of current module")
    label_sc3.place(x = 10, y = 30)
    self.node1Selection = ttk.Combobox(SpecialCommandSec, textvariable=self.nodeSelect1, width = 10)
    self.node1Selection['values'] = ()
    self.node1Selection.bind('<<ComboboxSelected>>',self.SelectNode1)
    self.node1Selection.place(x = 170, y = 30)
    label_sc4 = Label(SpecialCommandSec, text = "Module 2")
    label_sc4.place(x = 10, y = 60)
    self.module2Selection = ttk.Combobox(SpecialCommandSec, textvariable=self.module2Select, width = 10)
    self.module2Selection['values'] = ()
    self.module2Selection.bind('<<ComboboxSelected>>',self.SelectSecondModule)
    self.module2Selection.place(x = 90, y = 60)
    label_sc5 = Label(SpecialCommandSec, text = "Node of module 2")
    label_sc5.place(x = 10, y = 90)
    self.node2Selection = ttk.Combobox(SpecialCommandSec, textvariable=self.nodeSelect2, width = 10) 
    self.node2Selection['values'] = ()
    self.node2Selection.place(x = 130, y = 90)

    #---------------- Extra Information -------------------------
    ExtraInfoSec = ttk.Labelframe(f1, text='Extra Information ', width = 400, height = 90)
    ExtraInfoSec.place(x = 370, y = 300)
    label7 = Label(ExtraInfoSec, text='Condition ')
    label7.place(x = 10, y = 10)
    Condition = Entry(ExtraInfoSec, textvariable=self.condition, width = 12)
    Condition.place(x = 10, y = 35)
    label8 = Label(ExtraInfoSec, text='Dependency ')
    label8.place(x = 120, y = 10)
    self.Dependency = ttk.Combobox(ExtraInfoSec, textvariable=self.dependency, width = 12) #, command = self.checkConnectivity
    self.Dependency['values'] = ()
    # self.Dependency.bind('<<ComboboxSelected>>',self.UpdateJoint)
    self.Dependency.place(x = 120, y = 35)

    label9 = Label(ExtraInfoSec, text='Expected elapsed time ')
    label9.place(x = 240, y = 10)
    ElapsTime = Entry(ExtraInfoSec, textvariable=self.elapstime, width = 10)
    ElapsTime.place(x = 240, y = 35)
    Timeoption = ttk.Combobox(ExtraInfoSec, textvariable = self.elapsedTimeType, width =5)
    Timeoption['values'] = ('none','sec','msec')
    Timeoption.set('none')
    Timeoption.place(x = 330, y = 35)

    #--------------- Command Records ---------------------------
    label_f3_0 = Label(f3, text = "Select section: ")
    label_f3_0.place(x = 10, y = 5)
    self.keyFrame = ttk.Combobox(f3, textvariable=self.selectedkeyFrame, width = 10)
    self.keyFrame.place(x = 105, y = 5)
    self.keyFrame.bind('<<ComboboxSelected>>',self.UpdateFrameWindows)

    self.panedWindow = PanedWindow(f3)
    self.panedWindow.place(x = 0, y = 25, relheight = 0.95, width = window_width - 30)

    self.scollForCommands = Scrollbar(f3)
    self.scollForCommands.place(x = window_width - 30, y = 50, relheight = 0.90)

    #---------------- Save Path -------------------------------
    SavePathSec = ttk.Labelframe(f1, text='Save Path ', width = 400, height = 50)
    SavePathSec.place(x = 370, y = 390)
    Savepath = Entry(SavePathSec, textvariable=self.savepath, width = 38)
    self.savepath.set(self.savePath)
    Savepath.place(x = 10, y = 3)
    selectPath = Button(SavePathSec, text = 'Select', command = self.SelectSavePath)
    selectPath.place(x = 325, y = 0)

    #----------------- Command History --------------------------
    self.CommandHis = Listbox(f2, width=44, height=24,listvariable = self.commandhis)
    self.CommandHis.bind('<<ListboxSelect>>', self.ModifyHistory)
    CommandHisScroller = Scrollbar(f2, command=self.CommandHis.yview)
    self.CommandHis.config(yscrollcommand=CommandHisScroller.set)
    CommandHisScroller.place(x = 361, y = 40, height = 390)
    self.CommandHis.place(x = 10, y = 40)

    #----------------- Update Command --------------------------
    UpdateCommandSec = ttk.Labelframe(f2, text='Update Command ', width = 370, height = 90)
    UpdateCommandSec.place(x = 390, y = 40)
    CommandEntry = Entry(UpdateCommandSec, textvariable=self.selectedcommand, width = 42)
    CommandEntry.place(x = 10, y = 5)
    self.CommandUpdateBtn = Button(UpdateCommandSec, text = "Update", command = self.UpdateSingleGaitEntry, state = DISABLED)
    self.CommandUpdateBtn.place(x = 10, y = 35)
    self.CommandDeleteBtn = Button(UpdateCommandSec, text = "Delete", command = self.DeleteSingleGait, state = DISABLED)
    self.CommandDeleteBtn.place(x = 290, y = 35)

    #---------------- Frame Based Operation --------------------
    FrameOpSec = ttk.Labelframe(f2, text='Section Based Commands ', width = 370, height = 65)
    FrameOpSec.place(x = 390, y = 140)
    FramePlay = Button(FrameOpSec, text = "Play Current Section", state = DISABLED)
    FramePlay.place(x = 5, y = 10)
    self.FrameDelete = Button(FrameOpSec, text = "Delete All After Current Section", state = DISABLED)
    self.FrameDelete.place(x = 152, y = 10)
    # WarningLabel = Label(FrameOpSec, text = "Warning: Delete current frame will delte all the frames after current frame") # , font={"family":"Times", "size":8, "weight":"BOLD"}
    # WarningLabel.place(x = 10, y = 40)

    #--------------- Play All Button --------------------------
    PlayallButton = Button(f2, text = "Play all the sections")
    PlayallButton.place(x = window_width-Border_width-130, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Open File Button ---------------------------
    Openfile = Button(f1, text = "Open Configuration", command = self.AskOpenFile)
    Openfile.place(x = window_width-Border_width-130, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Open Existing Gait ---------------------------
    self.OpenGait = Button(f1, text = "Open Gait File", command = self.OpenGaitFile, state = DISABLED)
    self.OpenGait.place(x = window_width-Border_width-270, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Save Button --------------------------------
    self.saveButton = Button(f1, text="Save", command = self.SaveGaitTable, state = DISABLED)
    self.saveButton.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)
    self.saveButton2 = Button(f2, text="Save", command = self.SaveGaitTable, state = DISABLED)
    self.saveButton2.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Play Frame --------------------------------
    self.Playframe = Button(f1, text = "Play Section", command = self.PlayFrame)  #, state = DISABLED)
    self.Playframe.place(x = 125, y = window_height-Border_hieht-5, anchor = SW)

    #---------------- Reset Frame -------------------------------
    self.Resetframe = Button(f1,text = "Reset", command = self.Reset)
    self.Resetframe.place(x = 225, y = window_height-Border_hieht-5, anchor = SW)

    #---------------- Add Frame --------------------------------
    self.Addframe = Button(f1,text = "Save Section", command = self.SaveFrame, state = DISABLED)
    self.Addframe.place(x = 285, y = window_height-Border_hieht-5, anchor = SW)

    #----------------- Add Current Command ---------------------
    self.Addcommand = Button(f1,text = "Add Command", command = self.AddGaitEntry, state = DISABLED, width = 11)
    self.Addcommand.place(x = 5, y = window_height-Border_hieht-5, anchor = SW)
  ## Callback for close button
  # @param self object pointer
  def CloseWindow(self):
    if self.initflag==0 or self.initflag==2:
      self.communicator.stop()
    if self.initflag==0:
      self.rungzserver.terminate()
      try:
        self.rungzclient.terminate()
      except Exception, e:
        pass
      call(["pkill", "gzserver"])
      call(["pkill", "gzclient"])
    self.quit()
  ## Initializaed the section, don't be fooled by the name
  # @param self Object pointer
  def InitialFrame(self):
    self.currentFrame = Section("Section_0")
    self.currentFrame.RecordCurrentPosition(self.ModuleList)
    self.keyFrames = []
    self.keyFrames.append(self.currentFrame)
    self.DeleteAllWidgetsOnHisWindow()
    self.keyFrameList.append(self.currentFrame.FrameID)
    self.keyFrame["values"] = tuple(self.keyFrameList)
    self.selectedkeyFrame.set("Section_0")
    self.allCommands = []

#---------------- Open Configurations ----------------------
  ## Callback of Open Configuration button
  # @param self Object pointer
  def AskOpenFile(self):
    self.file_opt['title'] = 'Open Configuration File'
    filename = tkFileDialog.askopenfilename(**self.file_opt)
    if filename == "":
        return
    # filename = "/home/edward/.gazebo/models/SMORES8Jack/InitialConfiguration"

    # open file on your own
    if filename:
      print "Filename is : ",filename
      # configFile = open(filename, 'r')
      self.ReadInConfiguration(filename)
    self.OpenGait["state"] = NORMAL
  ## Parse the configuration file and build the configuration tree
  # @param self Objetc pointer
  # @param filename Name of the configuration file
  def ReadInConfiguration(self, filename):
    self.ModuleList = []
    self.modulenames = []
    self.ConnectionList = []
    self.tree = ET.parse(filename)
    root = self.tree.getroot()
    print "Root is: ",root.tag
    modules = root.find("modules")
    # self.modulenames = []
    for eachmodule in modules.findall('module') :
      modelname = eachmodule.find('name').text
      print "Module name: ",modelname
      jointanglestr = eachmodule.find('joints').text
      print "Joint angle: ",self.StringToList(jointanglestr)
      positionstr = eachmodule.find('position').text
      newmodule = Module(modelname, self.StringToList(jointanglestr),self.StringToList(positionstr))
      self.ModuleList.append(newmodule)
      self.modulenames.append(modelname)
    connections = root.find("connections")
    for eachconnection in connections.findall('connection') :
      model1name = eachconnection.find("module1").text
      model2name = eachconnection.find("module2").text
      node1 = int(eachconnection.find("node1").text)
      node2 = int(eachconnection.find("node2").text)
      newconnection = Connection(self.GetModuleByName(model1name),self.GetModuleByName(model2name),node1,node2)
      self.ConnectionList.append(newconnection)
      self.GetModuleByName(model1name).connection(node1,self.ConnectionList[-1])
      self.GetModuleByName(model2name).connection(node2,self.ConnectionList[-1])

    self.name['values'] = tuple(self.modulenames)
    if self.initflag==0 or self.initflag==2:
      self.SendLoadConfigurationMessage(filename)
    if self.initflag==0:
      try:
        self.rungzclient.terminate()
      except Exception, e:
        pass
      time.sleep(2)
      self.rungzclient = Popen(['gzclient'], stdout=PIPE)
    self.InitialFrame()
  ## Creates a open configuration request and publish to worldplugin
  # @param self Object pointer
  # @param configure_path Path of the configuration file
  def SendLoadConfigurationMessage(self,configure_path):
    newmessage = GaitRecMessage()
    newmessage.ModelName = "Module_0"
    newmessage.NewFrame = False
    newmessage.PlayStatus = False
    newmessage.LoadConfiguration = True
    newmessage.ExtrInfo = configure_path
    if self.initflag==0 or self.initflag==2:
      self.communicator.publish(newmessage)

#---------------- Module Selection ----------------------
  ## Callback of Select Model Combobox
  # @param self Object pointer
  # @param args Other arguments
  def UpdateJoint(self,*args):
    modelname = self.modelname.get()
    moduleObj = self.GetModuleByName(modelname)
    self.UpdateJointValue()
    # self.elapstime.set(0.0)
    self.jointAngleDifferenceTracking = [0]*4
    self.Addcommand["state"] = NORMAL
    self.ResetAssociateWindow()
    # self.Disconnect["state"] = NORMAL

#---------------- Utility functions --------------------
  ## Get module object by specifying the name
  # @param self Object pointer
  # @param modelname String: Name of the model
  # @return if there is a model return the model object, otherwise return False
  def GetModuleByName(self,modelname):
    for eachmodule in self.ModuleList:
      if eachmodule.ModelName == modelname:
        return eachmodule
    return False
  ## Refresh the dependency list
  # @param self Object pointer
  def RefreshDependencyList(self):
    self.Dependency['values'] = tuple(self.DependencyList)
  ## Convert a string that has multiple values to tuple
  # @param self Object pointer
  # @param anglestring A string that has values separated by spaces
  # @return Tuple of those values
  def StringToTuple(self, anglestring):
    jointangles = [float(x) for x in anglestring.split()]
    return tuple(jointangles)
  ## Convert a string that has multiple values to list
  # @param self Object pointer
  # @param anglestring A string that has values separated by spaces
  # @return List of those values
  def StringToList(self, anglestring):
    jointangles = [float(x) for x in anglestring.split()]
    return jointangles

#--------------- Add Gait Table ------------------------
  ## Callback for Add Command Button
  # @param self Object pointer
  def AddGaitEntry(self):
    if self.elapsedTimeType.get() == "sec" :
      currenttimer = int(self.elapstime.get()*1000)
    elif self.elapsedTimeType.get() == "msec" :
      currenttimer = int(self.elapstime.get())
    elif self.elapsedTimeType.get() == "none" :
      currenttimer = 0
    if not self.condition.get() in self.DependencyList and \
        len(self.condition.get()) > 0:
      self.DependencyList.append(self.condition.get())
      print "Dependency list: ",self.DependencyList
    if self.commandType.get() == 0:
      module_list_of_gait = []
      module_list_of_gait.append(self.modelname.get())
      for each_associate in self.frontWheelAssociates:
        if not each_associate.ModuleName in module_list_of_gait:
          module_list_of_gait.append(each_associate.ModuleName)
      for each_associate in self.lftWheelAssociates:
        if not each_associate.ModuleName in module_list_of_gait:
          module_list_of_gait.append(each_associate.ModuleName)
      for each_associate in self.rgtWheelAssociates:
        if not each_associate.ModuleName in module_list_of_gait:
          module_list_of_gait.append(each_associate.ModuleName)
      for each_associate in self.centralBendAssociates:
        if not each_associate.ModuleName in module_list_of_gait:
          module_list_of_gait.append(each_associate.ModuleName)
      #moduleObj = self.GetModuleByName(module_id)
      for each_module_name in module_list_of_gait:
        module_obj = self.GetModuleByName(each_module_name)
        jointsflags = [self.typeSelection.get()]*4
        newgaits = GaitEntry(each_module_name,module_obj.JointAngle[:],currenttimer, \
            self.dependency.get(),self.condition.get(),False,jointsflags)
        # self.CurrentFrameRec.append(newgaits)
        self.currentFrame.AddGaitToSection(newgaits)
        self.allCommands.append(newgaits.GaitToStr())
        self.commandhis.set(tuple(self.allCommands))
        self.AddGaitToCurrentFrame(newgaits)
    elif self.commandType.get() == 1:
      self.ConnectSend(currenttimer)
    elif self.commandType.get() == 2:
      self.DisconnectSend(currenttimer)
    self.RefreshDependencyList()
    #self.RefreshGaitRecorder()
    self.jointAngleDifferenceTracking = [0]*4
    self.name["state"] = NORMAL
    self.saveButton["state"] = NORMAL
    self.saveButton2["state"] = NORMAL
    self.Addframe["state"] = NORMAL
    self.Addcommand["state"] = DISABLED
    self.Resetframe["state"] = NORMAL
  ## Add gait to current section and display them in the Manage Gait Table tag
  # @param self Object pointer
  # @param gait_obj A gait object
  def AddGaitToCurrentFrame(self,gait_obj):
    if not gait_obj.ModuleName in self.currentFrame.ModulesInThisFrame:
      self.currentFrame.AddNewChangedModuleToFrame(gait_obj)
      new_rec_frame = Frame(self.panedWindow)
      if len(self.frameListForGaitRecord)>0 and \
          len(self.frameListForGaitRecord)<=8 and \
          len(self.frameListForGaitRecord)>len(self.frameButtonList):
        self.frameListForGaitRecord[-1].destroy()
      self.frameListForGaitRecord.append(new_rec_frame)
      self.panedWindow.add(self.frameListForGaitRecord[-1], minsize = 10, width = 100)
      new_button = Button(self.frameListForGaitRecord[-1], \
          text = gait_obj.ModuleName, height = 1, \
          command = lambda: self.ResizePanelSize(len(self.frameButtonList)+1))
      new_button.place(x = 0, y = 0, relwidth = 1)
      self.frameButtonList.append(new_button)
      list_var =StringVar()
      self.frameListVars.append(list_var)
      new_list = Listbox(self.frameListForGaitRecord[-1], \
          listvariable=self.frameListVars[-1])
      new_list.place(x = 0, y = 25, relwidth = 1, height = 460)
      new_list.config(yscrollcommand=self.scollForCommands.set)
      if len(self.currentFrame.ModulesInThisFrame) == 0:
        self.scollForCommands["command"] = new_list.yview
      self.frameListList.append(new_list)
      self.currentFrame.AddGaitToModule(gait_obj)
      self.frameListVars[-1].set(tuple(self.currentFrame.GaitStrListOfModule[-1]))
      # self.currentFrame.GaitStrListOfModule.append([gait_obj.GaitToStr()])
      if len(self.frameListForGaitRecord)<8:
        occupied_frame = Frame(self.panedWindow)
        self.frameListForGaitRecord.append(occupied_frame)
        self.panedWindow.add(self.frameListForGaitRecord[-1], minsize = 10)
    else:
      self.currentFrame.AddNewChangedModuleToFrame(gait_obj)
      list_idx = self.currentFrame.ModulesInThisFrame.index(gait_obj.ModuleName)
      # self.currentFrame.GaitStrListOfModule[list_idx].append(gait_obj.GaitToStr())
      self.currentFrame.AddGaitToModule(gait_obj,list_idx)
      gait_list = self.currentFrame.GaitStrListOfModule[list_idx]
      self.frameListVars[list_idx].set(tuple(gait_list))
  ## Send disconnect message to worldplugin
  # @param self Object pointer
  # @param time_interval How long this command will be executed
  def DisconnectSend(self, time_interval):
    newmessage = GaitRecMessage()
    newmessage.ModelName = self.modelname.get()
    newmessage.NewFrame = False
    newmessage.PlayStatus = True
    the_connection = self.GetModuleByName(self.modelname.get()).nodes[self.nodeSelect1.get()]
    connection_idx = self.ConnectionList.index(the_connection)
    del self.ConnectionList[connection_idx]
    self.GetModuleByName(self.modelname.get()).nodes[self.nodeSelect1.get()] = []
    self.GetModuleByName(self.module2Select.get()).node[self.nodeSelect2.get()] = []
    if len(self.module2Select.get())>0:
      newmessage.ExtrInfo = "$ - &"+self.modelname.get()+" &"+self.module2Select.get()
    else:
      newmessage.ExtrInfo = "$ - &"+self.modelname.get()+" "+"&X"
    if time_interval != 0 :
      newmessage.ExtrInfo += " [" + str(time_interval) +"]"
    if self.condition.get() != "" :
      newmessage.ExtrInfo += " {" + self.condition.get() +"}"
    if self.dependency.get() != "" :
      newmessage.ExtrInfo += " (" + self.dependency.get() +")"
    newmessage.ExtrInfo += " ;"
    if self.initflag==0 or self.initflag==2:
      self.communicator.publish(newmessage)
    newgaits = GaitEntry(self.modelname.get(),[0,0,0,0],self.elapstime.get(),self.dependency.get(),self.condition.get())
    newgaits.AddExtraInfo(newmessage.ExtrInfo)
    newgaits.SpecialEntry = True
    self.AddGaitToCurrentFrame(newgaits)
  ## Send connect message to worldplugin
  # @param self Object pointer
  # @param time_interval How long this command will be executed
  def ConnectSend(self, time_interval):
    if len(self.modelname.get())>0 and len(self.module2Select.get())>0 and len(self.nodeSelect1.get())>0 and len(self.nodeSelect2.get())>0:
      newmessage = GaitRecMessage()
      newmessage.ModelName = self.modelname.get()
      newmessage.NewFrame = False
      newmessage.PlayStatus = True
      new_connection = Connection(self.GetModuleByName(self.modelname.get()), \
          self.GetModuleByName(self.module2Select.get()),self.nodeSelect1.get(), \
          self.nodeSelect2.get())
      self.ConnectionList.append(new_connection)
      self.GetModuleByName(self.modelname.get()).nodes[self.nodeSelect1.get()] = self.ConnectionList[-1]
      self.GetModuleByName(self.module2Select.get()).node[self.nodeSelect2.get()] = self.ConnectionList[-1]
      newmessage.ExtrInfo = "$ + &"+self.modelname.get()+" #"+self.nodeSelect1.get()+" &"+self.module2Select.get()+" #"+self.nodeSelect2.get()
      if time_interval != 0 :
        newmessage.ExtrInfo += " [" + str(time_interval) +"]"
      if self.condition.get() != "" :
        newmessage.ExtrInfo += " {" + self.condition.get() +"}"
      if self.dependency.get() != "" :
        newmessage.ExtrInfo += " (" + self.dependency.get() +")"
      newmessage.ExtrInfo += " ;"
      if self.initflag==0 or self.initflag==2:
        self.communicator.publish(newmessage)
      newgaits = GaitEntry(self.modelname.get(),[0,0,0,0],self.elapstime.get(),self.dependency.get(),self.condition.get())
      newgaits.AddExtraInfo(newmessage.ExtrInfo)
      newgaits.SpecialEntry = True
      self.AddGaitToCurrentFrame(newgaits)
  ## Delets all the widgets on the second tag page
  # @param self Object pointer
  def DeleteAllWidgetsOnHisWindow(self):
    for each_widget in self.frameListForGaitRecord:
      each_widget.destroy()
    for each_widget in self.frameButtonList:
      each_widget.destroy()
    for each_widget in self.frameListList:
      each_widget.destroy()
    self.frameListVars = []
    self.scollForCommands["command"] = ()
  ## Click the button on the top to resize the panel size
  # It doesn't work
  # @param self Object pointer
  # @param button_idx Integer: index of the panel
  def ResizePanelSize(self, button_idx):
    print "Button index is ", button_idx
    self.panedWindow.paneconfigure(self.frameListForGaitRecord[button_idx], \
        width = 100)
  ## Converts gait object to string
  # @param self Object pointer
  # @param gaittableobj A GaitEntry object
  # @return Gait string
  def GaitObjToStr(self,gaittableobj):
    if gaittableobj.SpecialEntry :
      gaitstr = gaittableobj.ModuleName
      return gaitstr
    else:
      gaitstr = ""
      gaitstr += gaittableobj.ModuleName+" "
      for i in xrange(4):
        if gaittableobj.AngleFlags[i] == 0:
          gaitstr+= "p"
        elif gaittableobj.AngleFlags[i] == 1:
          gaitstr+= "s"
        elif gaittableobj.AngleFlags[i] == 2:
          gaitstr+= "t"        
        gaitstr+= str(gaittableobj.Joints[i])+" "
      if gaittableobj.Timer > 0:
        gaitstr+= "["+str(gaittableobj.Timer)+"] "
      if len(gaittableobj.condition_id) > 0 :
        gaitstr+= "{"+gaittableobj.condition_id+"} "
      if len(gaittableobj.dependency_id) > 0 :
        gaitstr+= "("+gaittableobj.dependency_id+") "
      gaitstr+=";"
      return gaitstr

#---------------- Save & Reset & Play Frames ---------------
  ## Saves current section, save section button callback
  # @param self Object pointer
  def SaveFrame(self):
    framenum = len(self.keyFrameList)
    while True:
      if not "Section_"+str(framenum) in self.DependencyList:
        break
      else:
        framenum += 1
    for each_indep_module in self.currentFrame.ModulesInThisFrame:
      for each_gait in reversed(self.currentFrame.GaitEntries):
        if each_gait.ModuleName == each_indep_module:
          if each_gait.condition_id == "":
            each_gait.condition_id = self.currentFrame.FrameID
          break
    self.currentFrame = Section("Section_"+str(framenum))
    self.currentFrame.RecordCurrentPosition(self.ModuleList)
    self.keyFrames.append(self.currentFrame)
    self.DeleteAllWidgetsOnHisWindow()
    self.keyFrameList.append(self.currentFrame.FrameID)
    self.keyFrame["values"] = tuple(self.keyFrameList)
    self.selectedkeyFrame.set("Section_"+str(framenum))
    self.DependencyList.append("Section_"+str(framenum-1))
    self.RefreshDependencyList()
    self.dependency.set("Section_"+str(framenum-1))
    self.Resetframe["state"] = DISABLED
    # self.keyFrameList.append(self.CurrentFrameRec)
    # self.CurrentFrameRec = []
    # self.RefreshGaitRecorder()
    self.UpdateFrameBox()
    self.SaveCurrentPose()
  ## Updates the section selction ComboBox
  # @param self Object pointer
  def UpdateFrameBox(self):
    self.allCommands = []
    for each_section in self.keyFrames:
      for each_entry in each_section.GaitEntries:
        self.allCommands.append(each_entry.GaitToStr())
    # self.Framename['values'] = tuple(frameliststr)
    self.commandhis.set(tuple(self.allCommands))
  ## Resets the current simulation world to the begin of the section
  # @param self Object pointer 
  def Reset(self):
    newmessage = GaitRecMessage()
    if self.currentFrame.GaitEntries:
      newmessage.ModelName = self.currentFrame.GaitEntries[0].ModuleName
    else:
      newmessage.ModelName = "Module_0"
    newmessage.NewFrame = False
    newmessage.PlayStatus = True
    newmessage.ResetFlag = True
    if self.initflag==0 or self.initflag==2:
      self.communicator.publish(newmessage)
    self.Playframe["state"] = NORMAL
    # self.Resetframe["state"] = DISABLED
    if self.commandType.get() == 0:
      self.GetModuleByName(self.modelname.get()).JointAngle[0] \
          -= self.jointAngleDifferenceTracking[0]
      for each_associate in self.frontWheelAssociates:
        self.GetModuleByName(each_associate.ModuleName).JointAngle[each_associate.Node] \
            -= self.jointAngleDifferenceTracking[0]*each_associate.Ratio*self.InterpretCorrelation(each_associate.corr)
      self.GetModuleByName(self.modelname.get()).JointAngle[1] \
          -= self.jointAngleDifferenceTracking[1]
      for each_associate in self.lftWheelAssociates:
        self.GetModuleByName(each_associate.ModuleName).JointAngle[each_associate.Node] \
            -= self.jointAngleDifferenceTracking[1]*each_associate.Ratio*self.InterpretCorrelation(each_associate.corr)
      self.GetModuleByName(self.modelname.get()).JointAngle[2] \
          -= self.jointAngleDifferenceTracking[2]
      for each_associate in self.rgtWheelAssociates:
        self.GetModuleByName(each_associate.ModuleName).JointAngle[each_associate.Node] \
            -= self.jointAngleDifferenceTracking[2]*each_associate.Ratio*self.InterpretCorrelation(each_associate.corr)
      self.GetModuleByName(self.modelname.get()).JointAngle[3] \
          -= self.jointAngleDifferenceTracking[3]
      for each_associate in self.centralBendAssociates:
        self.GetModuleByName(each_associate.ModuleName).JointAngle[each_associate.Node] \
            -= self.jointAngleDifferenceTracking[3]*each_associate.Ratio*self.InterpretCorrelation(each_associate.corr)
      self.UpdateJointValue()
    print "Reset message sent"
  ## Plays the current section
  # @param self Object pointer
  def PlayFrame(self):
    for eachgait in self.currentFrame.GaitEntries :
      if not eachgait.SpecialEntry:
        self.PublishMessage(eachgait,True)
      else:
        self.PublishMessageSpecial(eachgait,True)
    # self.Playframe["state"] = DISABLED
    print "All information published"
    self.Resetframe["state"] = NORMAL
  ## Generates and publish save current section command to worldplugin
  # @param self Object pointer
  def SaveCurrentPose(self):
    newmessage = GaitRecMessage()
    newmessage.ModelName = "SaveFrame"
    newmessage.NewFrame = True
    newmessage.PlayStatus = True
    if self.initflag==0 or self.initflag==2:
      self.communicator.publish(newmessage)
  ## Updates the panel window after selecting a section
  # @param self Object pointer
  # @param args Other variables
  def UpdateFrameWindows(self,*args):
    print "Runs update frame window"
    self.DeleteAllWidgetsOnHisWindow()
    frame_id = self.selectedkeyFrame.get()
    idx = self.keyFrameList.index(frame_id)
    print "Section index: ",idx
    self.currentFrame = self.keyFrames[idx]
    self.currentFrame.EmptyTheContainer()
    for each_gait in self.keyFrames[idx].GaitEntries:
      print "gait takes forever"
      self.AddGaitToCurrentFrame(each_gait)

#----------- Manually Update Section ----------------- 
  ## Callback when you click a gait entry in the third tag ("Gait Records")
  # @param self Object pointer
  # @param args Other variables
  def ModifyHistory(self,*args):
    self.historyidx = int(self.CommandHis.curselection()[0])
    print "Select item: ",self.historyidx
    self.selectedcommand.set(self.allCommands[self.historyidx])
    self.CommandDeleteBtn["state"] = NORMAL
    self.CommandUpdateBtn["state"] = NORMAL
  ## Update a GaitEntry object when you finish editing
  # Callback of the Update button in the third tag ("Gait Records")
  # @param self Object pointer
  def UpdateSingleGaitEntry(self):
    newsinglegait = self.selectedcommand.get()
    self.allCommands[self.historyidx] = newsinglegait
    updatedgait = self.InterpretGaitString(newsinglegait)
    # self.CurrentFrameHis[self.historyidx] = updatedgait
    (modified_gait,gaits,idx) = self.FindGaitEntryBasedOnIdx(self.historyidx)
    gaits[idx] = updatedgait

    self.UpdateFrameBox()
    self.saveButton["state"] = NORMAL
    self.saveButton2["state"] = NORMAL
  ## Delete a GaitEntry object after you hit the delete button
  # Callback of the Delete button in the third tag ("Gait Records")
  # @param self Object pointer
  def DeleteSingleGait(self):
    del self.allCommands[self.historyidx]
    (modified_gait,gaits,idx) = self.FindGaitEntryBasedOnIdx(self.historyidx)
    del gaits[idx]
    self.selectedcommand.set("")
    self.UpdateFrameBox()
    self.saveButton["state"] = NORMAL
    self.saveButton2["state"] = NORMAL
    self.CommandDeleteBtn["state"] = DISABLED
    self.CommandUpdateBtn["state"] = DISABLED
  ## Finds the GaitEntry object by index
  # for update and delete gait in the Gait Records window
  # @param self Object pointer
  # @param gait_idx Integer, the index number of the gait
  # @return (the GaitEntry object, Section GaitEntry List, And Section GaitEntry index)
  def FindGaitEntryBasedOnIdx(self,gait_idx):
    before_substr = gait_idx
    for each_section in self.keyFrames:
      after_substr = before_substr-len(each_section.GaitEntries)
      if before_substr>=0 and after_substr<0:
        return (each_section.GaitEntries[before_substr],each_section.GaitEntries, \
            before_substr)
      else:
        before_substr = after_substr

  # def CommandRecModify(self):
  #   self.recidx = int(self.CommandRec.curselection()[0])
  #   gaitentryobj = self.CurrentFrameRec[self.recidx]
  #   if gaitentryobj.ModuleName[0] == "-" or gaitentryobj.ModuleName[0] == "+" :
  #     tmpstring = gaitentryobj.ModuleName[2:]
  #     self.modelname.set(tmpstring[0:tmpstring.find(" ")])
  #     tmpstring = gaitentryobj.ModuleName[tmpstring.find(" ")+1:]
  #     self.othermodelname.set(tmpstring[0:tmpstring.find(" ")])
  #   else:
  #     self.Joint3.set(gaitentryobj.Joints[3]/PI*180)
  #     # self.front_angle.set(gaitentryobj.Joints[0]/PI*180)
  #     # self.left_angle.set(gaitentryobj.Joints[1]/PI*180)
  #     # self.right_angle.set(gaitentryobj.Joints[2]/PI*180)
  #     # self.group.set(gaitentryobj.Group)
  #     self.elapstime.set(gaitentryobj.Timer/1000.0)
  #     self.modelname.set(gaitentryobj.ModuleName)
  #     if gaitentryobj.AngleFlags[0] == 0:
  #       self.frontModA.select()
  #       self.frontangle.set(gaitentryobj.Joints[0]/PI*180)
  #       self.frontspeed.set(0)
  #     else:
  #       self.frontspeed.set(gaitentryobj.Joints[0])
  #       self.frontangle.set(0)
  #       self.frontModS.select()
  #     if gaitentryobj.AngleFlags[1] == 0:
  #       self.WheelModA.select()
  #       self.left_angle.set(gaitentryobj.Joints[1]/PI*180)
  #       self.right_angle.set(gaitentryobj.Joints[2]/PI*180)
  #       self.leftspeed.set(0)
  #       self.rightspeed.set(0)
  #     else:
  #       self.left_angle.set(0)
  #       self.right_angle.set(0)
  #       self.leftspeed.set(gaitentryobj.Joints[1])
  #       self.rightspeed.set(gaitentryobj.Joints[2])
  #       self.WheelModS.select()
  #   self.saveButton["state"] = NORMAL
  #   self.saveButton2["state"] = NORMAL

  # def CommandRecSave(self):
  #   gaitentryobj = self.CurrentFrameRec[self.recidx]
  #   if gaitentryobj.ModuleName[0] == "-" or gaitentryobj.ModuleName[0] == "+" :
  #     tmpstring = gaitentryobj.ModuleName[2:]
  #     namestring1 = gaitentryobj.ModuleName[2:tmpstring.find(" ")]
  #     tmpstring = gaitentryobj.ModuleName[tmpstring.find(" ")+1:]
  #     tmpstring = gaitentryobj.ModuleName[tmpstring.find(" ")+1:]
  #     gaitentryobj.ModuleName == gaitentryobj.ModuleName[0:2]+namestring1+" "+self.othermodelname.get()+" "+tmpstring
  #   else:
  #     gaitentryobj.Joints[3] = self.Joint3.get()/180.0*PI
  #     if self.frontmode.get() == 0 :
  #       gaitentryobj.AngleFlags[0] = 0
  #       gaitentryobj.Joints[0] = self.frontangle.get()/180.0*PI
  #     else:
  #       gaitentryobj.AngleFlags[0] = 1
  #       gaitentryobj.Joints[0] = self.frontspeed.get()
  #     if self.wheelmode.get() == 0 :
  #       gaitentryobj.AngleFlags[1] = 0
  #       gaitentryobj.AngleFlags[2] = 0
  #       gaitentryobj.Joints[1] = self.left_angle.get()/180.0*PI
  #       gaitentryobj.Joints[2] = self.right_angle.get()/180.0*PI
  #     else:
  #       gaitentryobj.AngleFlags[1] = 1
  #       gaitentryobj.AngleFlags[2] = 1
  #       gaitentryobj.Joints[1] = self.leftspeed.get()
  #       gaitentryobj.Joints[2] = self.rightspeed.get()
  #     # gaitentryobj.GroupIncr = self.group.get() - gaitentryobj.Group + gaitentryobj.GroupIncr
  #     # gaitentryobj.Group = self.group.get()
  #     gaitentryobj.Timer = int(self.elapstime.get()*1000)
  #   # self.RefreshGaitRecorder()

  # def CommandRecDelete(self):
  #   self.recidx = int(self.CommandRec.curselection()[0])
  #   del self.CurrentFrameRec[self.recidx]
  #   # self.RefreshGaitRecorder()
  #   self.saveButton["state"] = NORMAL
  #   self.saveButton2["state"] = NORMAL
  #   if len(self.CurrentFrameRec)>0:
  #     self.Addframe["state"] = NORMAL
  #   else:
  #     self.Addframe["state"] = DISABLED

  ## Publish common command message
  # Control joint position, speed or torque
  # @param self Object Pointer
  # @param eachgaittable A GaitEntry object
  # @param playstate Bool, True: for playmode, update model state using gait string; 
  #                        False: update model state immediately 
  def PublishMessage(self,eachgaittable,playstate):
    newmessage = GaitRecMessage()
    newmessage.ModelName = eachgaittable.ModuleName
    newmessage.NewFrame = False # self.Newframe
    newmessage.PlayStatus = playstate
    for i in xrange(4):
      newmessage.JointAngles.append(eachgaittable.Joints[i])
    # print newmessage.ModelName,":",newmessage.JointAngles
    newmessage.Timer = eachgaittable.Timer
    newmessage.Condition = eachgaittable.condition_id
    newmessage.Dependency = eachgaittable.dependency_id
    for i in xrange(4):
      newmessage.Flags.append(eachgaittable.AngleFlags[i])
    # print "Listeners are ",self.publisher.showlisteners()
    # self.newconnection.sendData(newmessage)
    # self.publisher.publish(newmessage)
    if self.initflag==0 or self.initflag==2:
      self.communicator.publish(newmessage)
    # eventlet.sleep(1.0)
    print "Information published"
  ## Publish special command messgae
  # Send connection and disconnection message
  # @param self Object Pointer
  # @param eachgaittable A GaitEntry object
  # @param playstate Bool, True: for playmode, update model state using gait string; 
  #                        False: update model state immediately 
  def PublishMessageSpecial(self,eachgaittable,playstate):
    newmessage = GaitRecMessage()
    newmessage.ModelName = eachgaittable.ModuleName
    newmessage.NewFrame = False # self.Newframe
    newmessage.PlayStatus = playstate
    # for i in xrange(4):
    #   newmessage.JointAngles.append(eachgaittable.Joints[i])
    newmessage.Timer = eachgaittable.Timer
    newmessage.Condition = eachgaittable.condition_id
    newmessage.Dependency = eachgaittable.dependency_id
    newmessage.ExtrInfo = eachgaittable.ExtraInfo
    # for i in xrange(3):
    #   newmessage.Flags.append(eachgaittable.AngleFlags[i])
    # print "Listeners are ",self.publisher.showlisteners()
    # self.newconnection.sendData(newmessage)
    # self.publisher.publish(newmessage)
    if self.initflag==0 or self.initflag==2:
      self.communicator.publish(newmessage)
    # eventlet.sleep(1.0)
    print "Information published"

#----------------- Save & Load Gait Table -------------------
  ## Write recorded gait table to file
  # @param self Object Pointer
  def SaveGaitTable(self):
    # Need a regular expression
    # commandpath = self.savepath.get()
    # if commandpath[-1] != "/":
    #   f = open(commandpath, 'w')
    # else:
    #   f = open(commandpath+"Commands", 'w')
    command_file_opt = {}
    command_file_opt['filetypes'] = [('smart gait table', '.gait'),('text files', '.txt'),('all files', '*')]
    command_file_opt["defaultextension"] = ".gait"
    command_file_opt['initialdir'] = self.savePath
    command_file_opt['parent'] = self.parent
    command_file_opt['title'] = 'Save Gait Command'
    f = tkFileDialog.asksaveasfile(mode='w', **command_file_opt)
    if f is None: # asksaveasfile return `None` if dialog closed with "cancel".
        return
    GaitStringList = []
    for eachframe in self.keyFrames:
      for eachentry in eachframe.GaitEntries:
        GaitStringList.append(eachentry.GaitToStr()+'\n')
    f.writelines(GaitStringList)
    f.close()
    print "Gait saved"
    self.saveButton["state"] = DISABLED
    self.saveButton2["state"] = DISABLED
  ## Open and read in a gait file
  # @param self Object Pointer
  def OpenGaitFile(self):
    self.file_opt['title'] = 'Open Gait Command'
    filename = tkFileDialog.askopenfilename(**self.file_opt)
    if filename == '':
        return
    # filename = "/home/edward/Simulation Plugins/GaitRecorder/pythonGUI/Commands"
    gaitfile = open(filename, 'r')
    # print gaitfile.readlines()
    self.currentFrame = Section("Section_0")
    self.currentFrame.RecordCurrentPosition(self.ModuleList)
    self.keyFrames = []
    self.keyFrames.append(self.currentFrame)
    self.DeleteAllWidgetsOnHisWindow()
    for eachlines in gaitfile.readlines():
      # print eachlines[0:-1]
      if eachlines.find("//") != -1:
        eachlines = eachlines[0:eachlines.find("//")]
      semicolon_index = eachlines.find(";")
      while semicolon_index != -1:
        newgait = self.InterpretGaitString(eachlines[0:semicolon_index])
        eachlines = eachlines[semicolon_index+1:]
        semicolon_index = eachlines.find(";")

        newcondition = newgait.condition_id
        if not newcondition in self.DependencyList and len(newcondition)>0:
          self.DependencyList.append(newcondition)
        self.currentFrame.AddGaitToSection(newgait)
        self.allCommands.append(newgait.GaitToStr())
        self.commandhis.set(tuple(self.allCommands))
        self.AddGaitToCurrentFrame(newgait)
      # print "Dependency list: ",self.DependencyList
    framenum = len(self.keyFrameList)
    while True:
      if not "Section_"+str(framenum) in self.DependencyList:
        break
      else:
        framenum += 1
    self.currentFrame.FrameID = "Section_"+str(framenum)
    self.keyFrameList = [self.currentFrame.FrameID]
    self.keyFrame["values"] = tuple(self.keyFrameList)
    self.selectedkeyFrame.set("Section_"+str(framenum))
    self.RefreshDependencyList()
    # self.RefreshGaitRecorder()
    self.saveButton["state"] = NORMAL
    self.saveButton2["state"] = NORMAL
    self.Addframe["state"] = NORMAL
  ## Convert a gait string to a GaitEntry object
  # @param self Object Pointer
  # @param gaitstring A gait command string
  # @return A GaitEntry object
  def InterpretGaitString(self,gaitstring):
    # print "gait string: ",gaitstring
    if gaitstring.find("[") != -1:
      timer = int(gaitstring[gaitstring.find("[")+1:gaitstring.find("]")])
    else:
      timer = 0
    if gaitstring.find("{") != -1:
      condition = gaitstring[gaitstring.find("{")+1:gaitstring.find("}")]
    else:
      condition = ""
    if gaitstring.find("(") != -1:
      dependency = gaitstring[gaitstring.find("(")+1:gaitstring.find(")")]
    else:
      dependency = ""
    if gaitstring.find("$") != -1 :
      and_idx = gaitstring.find("&")
      module_name = gaitstring[and_idx+1:gaitstring.find(" ",and_idx)]
      return GaitEntry(module_id = module_name,jointangles = [0,0,0,0], \
          timer = 0,dependency = dependency, condition = condition, \
          special = True, extra_info = gaitstring)
    else:
      gaitstring = gaitstring.strip()
      fields = gaitstring.split()
      # for x in range(len(gaitstring)):
      #   if gaitstring[x] != " ":
      #     start_idx = x
      #     break
      # idx = gaitstring.find(" ",start_idx)
      modelname = fields[0]
      # gaitstring = gaitstring[idx+1:]
      joints = []
      jointsflags = []
      for i in xrange(1,5):
        value_candidate = fields[i]
        if value_candidate[0] == "p":
          jointsflags.append(0)
        elif value_candidate[0] == "s" :
          jointsflags.append(1)
        elif value_candidate[0] == "t" :
          jointsflags.append(2)
        elif value_candidate[0] == "i" :
          jointsflags.append(3)
        elif value_candidate[0] == "c" :
          jointsflags.append(4)
        elif value_candidate[0] == "d" :
          jointsflags.append(5)
        if value_candidate[1:] == "":
          joints.append(0.0)
        else:
          joints.append(float(value_candidate[1:]))
      #   gaitstring = gaitstring[idx+1:]
      # # print "joint size: ", len(joints)
      return GaitEntry(modelname,joints,timer,dependency,condition, \
          False,jointsflags[0:4])

#----------------- Common Command Related -------------------
  ## Callback function of the scale bar moving event
  # Update the joint angle in simulator when slide the scale bar
  # @param self Object pointer
  # @args Other arguements
  def DynamicUpdate(self, *args):
    if len(self.modelname.get()) > 0 :
      message_queue = []
      newmessage = GaitRecMessage()
      newmessage.ModelName = self.modelname.get()
      newmessage.NewFrame = False
      newmessage.PlayStatus = False
      the_module = self.GetModuleByName(self.modelname.get())
      diff = self.valueSetting.get()/180.0*PI - \
          the_module.JointAngle[self.jointSelection.get()]
      the_module.JointAngle[self.jointSelection.get()] = \
          self.valueSetting.get()/180.0*PI
      for i in xrange(4):
        newmessage.JointAngles.append(the_module.JointAngle[i])
      message_queue.append(newmessage)
      for each_associates in self.currentAssociates:
        if not self.CheckTheExistingCommand(each_associates.ModuleName, message_queue):
          newmessage = GaitRecMessage()
          newmessage.ModelName = each_associates.ModuleName
          newmessage.NewFrame = False
          newmessage.PlayStatus = False
          the_module = self.GetModuleByName(each_associates.ModuleName)
          the_module.JointAngle[each_associates.Node] = \
              the_module.JointAngle[each_associates.Node] + \
              diff*each_associates.Ratio* \
              self.InterpretCorrelation(each_associates.Correlation)
          for i in xrange(4):
            newmessage.JointAngles.append(the_module.JointAngle[i])
          message_queue.append(newmessage)
        else:
          the_messgae = self.CheckTheExistingCommand( \
              each_associates.ModuleName, message_queue)
          the_module = self.GetModuleByName(the_messgae.ModelName)
          the_module.JointAngle[each_associates.Node] = \
              the_module.JointAngle[each_associates.Node] + \
              diff*each_associates.Ratio* \
              self.InterpretCorrelation(each_associates.Correlation)
          for i in xrange(4):
            newmessage.JointAngles.append(the_module.JointAngle[i])
      if self.initflag==0 or self.initflag==2:
        for each_message in message_queue:
          self.communicator.publish(each_message)
      print "Angle Updating"
      self.Addcommand["state"] = NORMAL
      self.jointAngleDifferenceTracking[self.jointSelection.get()] += diff
      if sum([abs(x) for x in self.jointAngleDifferenceTracking]) > 1.0/180.0*PI:
        print "Angle difference is ", diff
        self.name["state"] = DISABLED
      else:
        self.name["state"] = NORMAL
      if self.jointSelection.get() != 3 and self.typeSelection.get() == 0:
        if self.valueSetting.get() == self.valueSetting["to"]:
          self.valueSetting["to"] = self.valueSetting["to"] + 90
          self.valueSetting["from"] = self.valueSetting["from"] + 90;
          self.valueSetting.set(self.valueSetting["to"] - 180)
          time.sleep(0.3)
        if self.valueSetting.get() == self.valueSetting["from"]:
          self.valueSetting["to"] = self.valueSetting["to"] - 90
          self.valueSetting["from"] = self.valueSetting["from"] - 90
          self.valueSetting.set(self.valueSetting["from"] + 180)
          time.sleep(0.3)
        self.valueInBox.set(self.valueSetting.get())
      if self.jointSelection.get() == 3 and self.typeSelection.get() == 0:
        self.valueInBox.set(self.valueSetting.get())
  ## Callback function binded to the entry box next to scale with enter key
  # @param self Object pointer
  # @param args Other arguements
  def UpdateFromValueBox(self, *args):
    if len(self.modelname.get()) > 0 :
      message_queue = []
      newmessage = GaitRecMessage()
      newmessage.ModelName = self.modelname.get()
      newmessage.NewFrame = False
      newmessage.PlayStatus = False
      the_module = self.GetModuleByName(self.modelname.get())
      diff = self.valueInBox.get()/180.0*PI - \
          the_module.JointAngle[self.jointSelection.get()]
      the_module.JointAngle[self.jointSelection.get()] = \
          self.valueInBox.get()/180.0*PI
      for i in xrange(4):
        newmessage.JointAngles.append(the_module.JointAngle[i])
      message_queue.append(newmessage)
      for each_associates in self.currentAssociates:
        if not self.CheckTheExistingCommand(each_associates.ModuleName, message_queue):
          newmessage = GaitRecMessage()
          newmessage.ModelName = each_associates.ModuleName
          newmessage.NewFrame = False
          newmessage.PlayStatus = False
          the_module = self.GetModuleByName(each_associates.ModuleName)
          the_module.JointAngle[each_associates.Node] = \
              the_module.JointAngle[each_associates.Node] + \
              diff*each_associates.Ratio* \
              self.InterpretCorrelation(each_associates.Correlation)
          for i in xrange(4):
            newmessage.JointAngles.append(the_module.JointAngle[i])
          message_queue.append(newmessage)
        else:
          the_messgae = self.CheckTheExistingCommand( \
              each_associates.ModuleName, message_queue)
          the_module = self.GetModuleByName(the_messgae.ModelName)
          the_module.JointAngle[each_associates.Node] = \
              the_module.JointAngle[each_associates.Node] + \
              diff*each_associates.Ratio* \
              self.InterpretCorrelation(each_associates.Correlation)
          for i in xrange(4):
            newmessage.JointAngles.append(the_module.JointAngle[i])
      if self.initflag==0 or self.initflag==2:
        for each_message in message_queue:
          self.communicator.publish(each_message)
      print "Angle Updating"
      self.Addcommand["state"] = NORMAL
      self.jointAngleDifferenceTracking[self.jointSelection.get()] += diff
      if sum([abs(x) for x in self.jointAngleDifferenceTracking]) > 1.0/180.0*PI:
        print "Angle difference is ", diff
        self.name["state"] = DISABLED
      else:
        self.name["state"] = NORMAL
      if self.typeSelection.get() == 0:
        if self.jointSelection.get() != 3:
          if (self.valueInBox.get() >= self.valueSetting["to"]) or \
              (self.valueInBox.get() <= self.valueSetting["from"]):
            self.valueSetting["to"] = self.valueInBox.get() + 180
            self.valueSetting["from"] = self.valueInBox.get() - 180
          self.valueSetting.set(self.valueInBox.get())
        else:
          if self.valueInBox.get() > 90:
            self.valueInBox.set(90)
          if self.valueInBox.get() < -90:
            self.valueInBox.set(-90)
          self.valueSetting.set(self.valueInBox.get())
  ## Check whether there is a command of the same model in a command list
  # @param self Object pointer
  # @param model_name Name string of the model
  # @param command_queue A command list
  # @return If found a command, then return the messga object; otherwise return False
  def CheckTheExistingCommand(self,model_name,command_queue):
    for each_command in command_queue:
      if each_command.ModelName == model_name:
        return each_command
    return False
  ## Convert a correlation boolean value to number that can be used in calculation
  # @param self Object pointer
  # @param corr Bool value stored in AssociateJ?oint object
  # @return Position correlation: 1.0 ; negtive correlation: -1.0
  def InterpretCorrelation(self, corr):
    if corr:
      return 1.0
    else:
      return -1.0

#---------------- Associates Related -----------------------
  ## Clear the association window
  # @param self Object pointer
  def ResetAssociateWindow(self):
    self.frontWheelAssociates = []
    self.lftWheelAssociates = []
    self.rgtWheelAssociates = []
    self.centralBendAssociates = []
    self.currentAssociates = []
    self.associatedJointsList.set(())
  ## Refresh association window
  # @param self Object pointer
  def RefreshAssociates(self):
    associate_list = []
    if self.jointSelection.get() == 0:
      self.currentAssociates = self.frontWheelAssociates
    if self.jointSelection.get() == 1:
      self.currentAssociates = self.lftWheelAssociates
    if self.jointSelection.get() == 2:
      self.currentAssociates = self.rgtWheelAssociates
    if self.jointSelection.get() == 3:
      self.currentAssociates = self.centralBendAssociates      
    for each_associate in self.currentAssociates:
      associate_list.append(each_associate.ToString())
    self.associatedJointsList.set(tuple(associate_list))
  ## Delete an association
  # @param self Object pointer
  def DeleteAnAssociate(self):
    if self.associatedJoints.curselection():
      self.associateIdx = int(self.associatedJoints.curselection()[0])
      if self.jointSelection.get() == 0:
        del self.frontWheelAssociates[self.associateIdx]
        self.currentAssociates = self.frontWheelAssociates
      if self.jointSelection.get() == 1:
        del self.lftWheelAssociates[self.associateIdx]
        self.currentAssociates = self.lftWheelAssociates
      if self.jointSelection.get() == 2:
        del self.rgtWheelAssociates[self.associateIdx]
        self.currentAssociates = self.rgtWheelAssociates
      if self.jointSelection.get() == 3:
        del self.centralBendAssociates[self.associateIdx]
        self.currentAssociates = self.centralBendAssociates
      self.RefreshAssociates()

#---------------- Update Frames ----------------------------
  # def UpdateFrame(self):


#---------------- Select File Save Path --------------------
  ## Select gaitfile save path
  # @param self Object pointer
  def SelectSavePath(self):
    select_dir_options = {}
    select_dir_options['initialdir'] = '~/'
    select_dir_options['parent'] = self.parent
    select_dir_options['title'] = 'Select Save Path'
    self.savePath = tkFileDialog.askdirectory(**select_dir_options)
    if self.savePath == "":
        return
    self.savepath.set(self.savePath+"/")

#---------------- Add Common Command -----------------------
  ## Open an add association window
  # @param self Object pointer
  def AddAssociates(self):
    asc = AddAssociate(self)
    self.wait_window(asc)
  ## Update joint value on the scale bar and entry next to it
  # @param self Object pointer
  def UpdateJointValue(self):
    node_idx = self.jointSelection.get()
    value_type = self.typeSelection.get()
    a_module = self.GetModuleByName(self.modelname.get())
    if node_idx == 3 and value_type == 0:
      self.valueSetting["from_"] = -90
      self.valueSetting["to"] = 90
    if node_idx != 3 and value_type == 0:
      if a_module.JointAngle[node_idx]/PI*180 > -180 and \
          a_module.JointAngle[node_idx]/PI*180 < 180:
        self.valueSetting["from_"] = -180
        self.valueSetting["to"] = 180
      else:
        self.valueSetting["from_"] = a_module.JointAngle[node_idx]/PI*180-180
        self.valueSetting["to"] = a_module.JointAngle[node_idx]/PI*180+180
    if a_module:
      self.valueSetting.set(a_module.JointAngle[node_idx]/PI*180)
      self.valueInBox.set(a_module.JointAngle[node_idx]/PI*180)
    self.RefreshAssociates()
  ## Callback of common command select radiobutton
  # @param self Object pointer
  def SelectCommonCommand(self):
    self.DisableSpecialCommand()
    self.EnableCommonCommand()
    self.ResetSpecialComand()
  ## Disable common command
  # @param self Object pointer
  def DisableCommonCommand(self):
    self.bend_joint["state"] = DISABLED
    self.left_joint["state"] = DISABLED
    self.right_joint["state"] = DISABLED
    self.front_joint["state"] = DISABLED
    self.modeAngle["state"] = DISABLED
    # self.modeSpeed["state"] = DISABLED
    # self.modeTorque["state"] = DISABLED
    self.associatedJoints["state"] = DISABLED
    self.ascModify["state"] = DISABLED
    self.ascDelete["state"] = DISABLED
    self.valueSetting["state"] = DISABLED
  ## Enable common command
  # @param self Object pointer
  def EnableCommonCommand(self):
    self.bend_joint["state"] = NORMAL
    self.left_joint["state"] = NORMAL
    self.right_joint["state"] = NORMAL
    self.front_joint["state"] = NORMAL
    self.modeAngle["state"] = NORMAL
    # self.modeSpeed["state"] = NORMAL
    # self.modeTorque["state"] = NORMAL
    self.associatedJoints["state"] = NORMAL
    self.ascModify["state"] = NORMAL
    self.ascDelete["state"] = NORMAL
    self.valueSetting["state"] = NORMAL

#---------------- Add Special Command ----------------------
  ## Callback of select connection radiobutton
  # @param self Object pointer
  def SelectConnection(self):
    self.ResetSpecialComand()
    self.EnableSpecialCommand()
    self.DisableCommonCommand()
    self.ConnectableModules = []
    for eachmodule in self.ModuleList:
      has_unoccupied_node = False
      for x in xrange(4):
        if len(eachmodule.nodes[x]) == 0:
          has_unoccupied_node = True
          break
      if has_unoccupied_node:
        self.ConnectableModules.append(eachmodule.ModelName)
    self.module2Selection['values'] = tuple(self.ConnectableModules)
    module1_name = self.modelname.get()
    node1_list = self.UpdateConnectableNodes(module1_name)
    self.node1Selection['values'] = tuple(node1_list)
    self.node2Selection["values"] = ()
  ## Select the second module which the current module connect to or disconnect to
  # @param self Object pointer
  # @param args Other arguments
  def SelectSecondModule(self,*args):
    if self.commandType.get() == 1:
      module2_name = self.module2Select.get()
      node2_list = self.UpdateConnectableNodes(module2_name)
      self.node2Selection['values'] = tuple(node2_list)
    elif self.commandType.get() == 2:
      module1_name = self.modelname.get()
      module2_name = self.module2Select.get()
      module_1 = self.GetModuleByName(module1_name)
      for x in xrange(4):
        if len(module_1.nodes[x]) > 0:
          if module_1.nodes[x].Module1.ModelName == module2_name:
            # self.node1Selection['values'] = (module_1.nodes[x].Node2)
            self.node2Selection['values'] = (module_1.nodes[x].Node1)
            self.nodeSelect1.set(module_1.nodes[x].Node2)
            self.nodeSelect2.set(module_1.nodes[x].Node1)
            break
          if module_1.nodes[x].Module2.ModelName == module2_name:
            # self.node1Selection['values'] = (module_1.nodes[x].Node1)
            self.node2Selection['values'] = (module_1.nodes[x].Node2)
            self.nodeSelect1.set(module_1.nodes[x].Node1)
            self.nodeSelect2.set(module_1.nodes[x].Node2)
            break
  ## Select the node of the first module
  # @param self Object pointer
  # @param args Other arguments
  def SelectNode1(self,*args):
    if self.commandType.get() == 2:
      module1_name = self.modelname.get()
      node_1 = self.nodeSelect1.get()
      module_1 = self.GetModuleByName(module1_name)
      module_1_name = module_1.nodes[node_1].Module1.ModelName
      module_2_name = module_1.nodes[node_1].Module2.ModelName
      if module_1_name != module1_name:
        self.module2Select.set(module_1_name)
        self.nodeSelect2.set(module_1.nodes[node_1].Node1)
        self.node2Selection['values'] = (module_1.nodes[node_1].Node1)
      if module_2_name != module1_name:
        self.module2Select.set(module_2_name)
        self.nodeSelect2.set(module_1.nodes[node_1].Node2)
        self.node2Selection['values'] = (module_1.nodes[node_1].Node2)
  ## Update the available nodes of the other module the current module connect to
  # @param self Object pointer
  # @param module_name Name string of the other module
  # @return A list of interger, which are the id of the available nodes
  def UpdateConnectableNodes(self, module_name):
    a_module = self.GetModuleByName(module_name)
    node_available = []
    for x in xrange(4):
      if len(a_module.nodes[x]) == 0:
        node_available.append(str(x))
    return node_available
  ## Reset the module and nodes selection in the special command section
  # @param self Object pointer
  def ResetSpecialComand(self):
    self.module2Select.set("")
    self.nodeSelect1.set("")
    self.nodeSelect2.set("")
  ## Callback of select disconnection radiobutton
  # @param self Object pointer
  def SelectDisconnection(self):
    self.ResetSpecialComand()
    self.EnableSpecialCommand()
    self.DisableCommonCommand()
    module_1 = self.GetModuleByName(self.modelname.get())
    self.DisconnectableModules = []
    self.DisconnectableNodes = []
    if module_1:
      for x in xrange(4):
        if len(module_1.nodes[x]) > 0:
          module1_name = module_1.nodes[x].Module1.ModelName
          module2_name = module_1.nodes[x].Module2.ModelName
          if module1_name != self.modelname.get():
            self.DisconnectableModules.append(module1_name)
          if module2_name != self.modelname.get():
            self.DisconnectableModules.append(module2_name)
          self.DisconnectableNodes.append(x)
    self.module2Selection['values'] = tuple(self.DisconnectableModules)
    self.node1Selection["values"] = tuple(self.DisconnectableNodes)
    self.node2Selection["values"] = ()
  ## Disable special command section
  # @param self Object pointer
  def DisableSpecialCommand(self):
    self.node1Selection["state"] = DISABLED
    self.module2Selection["state"] = DISABLED
    self.node2Selection["state"] = DISABLED
  ## Enable special command section
  # @param self Object pointer
  def EnableSpecialCommand(self):
    self.node1Selection["state"] = NORMAL
    self.module2Selection["state"] = NORMAL
    self.node2Selection["state"] = NORMAL

## A class for adding associates dialog
class AddAssociate(Toplevel):
  ## Constructor
  # @param self Object pointer
  # @param parent Parent widget, which is the python gui window
  def __init__(self, parent):
    Toplevel.__init__(self,parent)
    ## Inherit from Toplevel object
    self.transient(parent)
    ## Title object of Toplevel object
    self.title("Add association")
    ## Parent, which is the class GaitRecorder
    self.parent = parent
    ## Variable stores the module name list for selecting
    self.moduleList = StringVar()
    ## Integer, Joint identity number
    self.jointSelection = IntVar()
    ## Integer, correlation type, -1 for negtive, 1 for positive
    self.corelation = IntVar()
    ## Correlation ratio
    self.ratio = DoubleVar()

    body = Frame(self, width = DIALOG_WIDTH, height = DIALOG_HEIGHT)
    ## Pointer to the object that needs to be focused initially
    self.initial_focus = self.body(body)
    body.pack(padx=5, pady=5)

    if not self.initial_focus:
      self.initial_focus = self

    self.protocol("WM_DELETE_WINDOW", self.cancel)
    self.geometry("%dx%d" % (DIALOG_WIDTH,
                            DIALOG_HEIGHT))
    self.initial_focus.focus_set()
    # self.wait_window(self)
  ## Dialog UI boday
  # @param self Object pointer
  # @param master Body's frame object
  # @return moduleName Combobox object
  def body(self, master):
    # create dialog body.  return widget that should have
    # initial focus.  this method should be overridden
    cancel = Button(master, text="Cancel", command = self.cancel, width = 6)
    cancel.place(x = DIALOG_WIDTH-20, y = DIALOG_HEIGHT-20, anchor = SE)
    add = Button(master, text="Add", width = 6, command = self.Add)
    add.place(x = 10, y = DIALOG_HEIGHT-20, anchor = SW)

    label1 = Label(master, text = "1. Select module:")
    label1.place(x = 10, y = 10)
    moduleName = ttk.Combobox(master,width = 10, textvariable = self.moduleList)
    moduleName["values"] = tuple(self.parent.modulenames)
    moduleName.place(x = 120, y = 10)

    label3 = Label(master, text = "2. Select joint:")
    label3.place(x = 10, y = 40)
    bard = Image.open("SmallSmores.png")
    bardejov = ImageTk.PhotoImage(bard)
    label2 = Label(master, image = bardejov)
    label2.image = bardejov
    label2.place(x=90, y=90)
    ## Ratio button for bend joint
    self.bend_joint = Radiobutton(master, text='Central Bending', variable=self.jointSelection, value=3)
    ## Ratio button for left joint
    self.left_joint = Radiobutton(master, text='Lft Wheel', variable=self.jointSelection, value=1)
    ## Ratio button for right joint
    self.right_joint = Radiobutton(master, text='Rgt Wheel', variable=self.jointSelection, value=2)
    ## Ratio button for front joint
    self.front_joint = Radiobutton(master, text='Front Wheel', variable=self.jointSelection, value=0)
    self.front_joint.select()
    self.bend_joint.place(x= 85, y = 65)
    self.front_joint.place(x= 85, y = 180)
    self.right_joint.place(x= 45, y = 140,anchor = CENTER)
    self.left_joint.place(x= 215, y = 140,anchor = CENTER)

    label4 = Label(master, text = "3. Select correlation")
    label4.place(x = 260, y = 40)
    ## Ratio button for positive correlation
    self.positiveCor = Radiobutton(master, text = "Positive", variable = self.corelation, value = 0)
    self.positiveCor.place(x = 270, y = 70)
    ## Ratio button for negative correlation
    self.negativeCor = Radiobutton(master, text = "Negative", variable = self.corelation, value = 1)
    self.negativeCor.place(x = 270, y = 100)

    label5 = Label(master, text = "4. Select ratio")
    label5.place(x = 260, y = 150)
    ## Entry box for correlation
    self.correlationRatio = Entry(master, textvariable = self.ratio, width = 10)
    self.ratio.set(1)
    self.correlationRatio.place(x = 270, y = 180)

    return moduleName
  ## Add button callback, which will vreate an associate in parent object
  # @param self Object pointer
  def Add(self):
    if len(self.moduleList.get()) > 0 and len(self.parent.modelname.get()) > 0:
      corr = True
      if self.corelation.get() == 0:
        corr = True
      if self.corelation.get() == 1:
        corr = False
      newAssociate = AssociateJoint(self.moduleList.get(),self.jointSelection.get(), \
          corr, self.ratio.get())
      if self.parent.jointSelection.get() == 0:
        self.parent.frontWheelAssociates.append(newAssociate)
        self.parent.currentAssociates = self.parent.frontWheelAssociates
      if self.parent.jointSelection.get() == 1:
        self.parent.lftWheelAssociates.append(newAssociate)
        self.parent.currentAssociates = self.parent.lftWheelAssociates
      if self.parent.jointSelection.get() == 2:
        self.parent.rgtWheelAssociates.append(newAssociate)
        self.parent.currentAssociates = self.parent.rgtWheelAssociates
      if self.parent.jointSelection.get() == 3:
        self.parent.centralBendAssociates.append(newAssociate)
        self.parent.currentAssociates = self.parent.centralBendAssociates
      self.parent.RefreshAssociates()
    self.parent.focus_set()
    self.destroy()
  ## Cancle adding associate
  # @param self Object pointer
  # @param event Event object  
  def cancel(self, event=None):
    # put focus back to the parent window
    self.parent.focus_set()
    self.destroy()
## Main function for this module, which will start the python gui
# @param falg Integer, 0 for normal mode, 1 for gui debug mode, 2 for python only mode
def main(flag):
  
  root = Tk()
  root.geometry(str(window_width)+"x"+str(window_height))
  root.wm_attributes("-topmost", 1)
  app = GaitRecorder(root,flag)
  root.mainloop()  


if __name__ == '__main__':
  if len(sys.argv) < 2:
    flag = 0
  else:
    if sys.argv[1] == '-gd': # gui debug mode, open gui only
      flag = 1
    if sys.argv[1] == '-o':  # do not invoke simulation
      flag = 2
  main(flag)  
#--------------- System related ----------------------
import time
from subprocess import call, Popen, PIPE
import sys
import os
from os.path import expanduser
#--------------- GUI Modules -------------------------
from Tkinter import *
import ttk
import tkFileDialog
from PIL import Image, ImageTk  # need to install: sudo apt-get install python-imaging-tk
#--------------- Representation related --------------
from Module import *
from Connection import *
import xml.etree.ElementTree as ET   # XML parser
#--------------- Communiation related ----------------
sys.path.append("../../Util/python_util")
from config_message_pb2 import *
# import eventlet  # need to install: $:sudo pip install eventlet
# from pygazebo import *  #need to install: $: sudo pip install pygazebo
# from gztopic import *
from gztopic_multithreading import *
#--------------- Kinematic related -------------------
from SimpleKL import CloseEnough, Connectable
import kinematics
# from SmoresKinematics import SmoresKinematics # Kinematics using Embedding code
#--------------- Mathematic related ------------------
from numpy import pi
#--------------- Debuggin Tools ----------------------
import pdb

window_width = 800
window_height = 520
Border_width = 20
Border_hieht = 40
PI = pi           #3.1415926 this value is close to the pi they used in the simulation
## Configuration editor python gui
class ConfigEditor(Frame):
  ## Constructor
  # @param self Object pointer
  # @param parent Parent object, which is tk root
  # @param flg Start flag, see GaitRecorder
  def __init__(self, parent, flag):
    Frame.__init__(self, parent)   
    
    self.DEFAULT_TYPE = 'muscle'
    self.DEFAULT_PATH = 'SMORE.sdf'
    #------------ Variables Initialization -------------------
    self.parent = parent
    self.ModuleList = []
    self.ConnectionList = []
    self.ModuleNameList = []
    self.nameTrash = []
    self.modelname = StringVar()
    self.current_model_type = self.DEFAULT_TYPE
    self.current_suggest_name = "Module"
    self.current_modeule_path = self.DEFAULT_PATH
    self.modelname.set('Module_0')
    self.InsertMethod = StringVar()
    self.InsertMethod.set('Connection')
    self.connectedmodelvar = StringVar()
    self.Node1 = IntVar()
    self.Node1.set(3)
    self.Node2 = IntVar()
    self.Node2.set(3)
    self.X_coor = DoubleVar()
    self.Y_coor = DoubleVar()
    self.Z_coor = DoubleVar()
    self.C_dis = DoubleVar()
    self.C_dis.set(0)
    # self.a_dis = DoubleVar()
    # self.a_dis.set(0)
    self.modellist = StringVar()
    self.adjacentnode = StringVar()
    self.savepathstr = StringVar()
    self.del_confirm = IntVar()
    self.initflag = flag
    self.ServerConnected = 0
    self.home = expanduser("~")

    #------------ File Options -------------------------------
    self.file_opt = options = {}
    # options['defaultextension'] = '.txt'
    options['filetypes'] = [('all files', '*'), ('Configuration file files', '.conf')]
    options['initialdir'] = '~/'
    options['parent'] = parent
    options['title'] = 'Open Configuration File'

    #------------ Run Simulation and GUI ---------------------
    if flag == 0: 
      self.rungzserver = Popen(['sh', 'RunSimulation.sh'], stdout=PIPE)
      time.sleep(0.3)
      self.rungzclient = Popen(['gzclient', '-g','libsystem_gui.so'], stdout=PIPE)
      time.sleep(2)

    #------------ Building Connections ----------------------
    if flag == 0 or flag == 2:
      self.communicator = GzCommunicator()
      self.communicator.start()
      self.configPub = self.communicator.CreatePulisher(
          "/gazebo/Configuration/configSubscriber",'config_message.msgs.ConfigMessage')
      # self.communicator.StartCommunicator("/gazebo/Configuration/configSubscriber",'config_message.msgs.ConfigMessage')
      self.ServerConnected = 1
      #++++++++++ These lines using a pygazebo library ++++++
      #++++++++++ Can only be used in gazebo 2.2+ +++++++++++
      # self.manager = Manager()
      # self.publisher = self.manager.advertise('/gazebo/Configuration/configSubscriber','config_message.msgs.ConfigMessage')  # 'config_message.msgs.ConfigMessage' /gazebo/Configuration/configSubscriber
      # self.publisher.wait_for_listener()

    #------------ Initializae GUI ---------------------------
    self.initUI()
    self.DisableInsertByConnection()
  ## Python ui initialization
  # @param self Object pointer    
  def initUI(self):
    
    self.parent.title("Configuration Generator")
    self.pack(fill=BOTH, expand=1)

    # --------------- Tabs --------------------------------------
    n = ttk.Notebook(self)
    f1 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED,); # first page, which would get widgets gridded into it
    f2 = Frame(n,height=window_height-Border_hieht,width=window_width-Border_width,relief=RAISED,); # second page
    n.add(f1, text='New Module')
    n.add(f2, text='Modify Old Module')
    n.pack()

    # --------------- Close Button ------------------------------
    closeButton = Button(f1, text="Close")
    closeButton["command"] = self.CloseWindow
    closeButton.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)
    closeButton2 = Button(f2, text="Close")
    closeButton2["command"] = self.CloseWindow
    closeButton2.place(x = window_width-Border_width-5, y = window_height-Border_hieht-5, anchor = SE)

    #---------------- Model Name ---------------------------------
    label = Label(f1, text='ModelName: ')
    label.place(x = 10, y = 5)
    name = Entry(f1, textvariable=self.modelname, width = 15)
    name.place(x = 90, y = 5)
    label_type = Label(f1, text='Model Type: ')
    label_type.place(x = 230, y = 5)
    self.type_name = Label(f1, text=self.current_model_type)
    self.type_name.place(x = 310, y = 5)
    selectModelButton = Button(f1, text="Select Model Type", command = self.SelectModelType)
    selectModelButton.place(x = 370, y = 2)
    defaultButton = Button(f1,text="Default", command = self.SetTypeDefault)
    defaultButton.place(x = 506, y = 2)

    #--------------- Insert by Connection configuration box ------
    insertByConn = ttk.Labelframe(f1, text='Insert By Connecting to Existing Models', width = 580, height = 240)
    insertByConn.place(x = 5,y = 30)

    #--------------- Insert by Connection configuration box ------
    insertByPos = ttk.Labelframe(f1, text='Insert By Position', width = 180, height = 240)
    insertByPos.place(x = 590,y = 30)

    #---------------- Radio selection -------------------------
    self.Rel_pos = Radiobutton(insertByConn, text='Connection', variable=self.InsertMethod, value='Connection', command = self.DisableXYZInput)
    self.Abs_pos = Radiobutton(insertByPos, text='Position', variable=self.InsertMethod, value='Position', command = self.EnableXYZInput)
    self.Abs_pos.select()  # this line doesn't work, haven't figure out why
    self.Rel_pos.place(x = 8, y = 2)
    self.Abs_pos.place(x = 8, y = 2)

    #--------------- Connect to -------------------------------
    label2 = Label(insertByConn, text='Connect to:')
    label2.place(x = 205, y = 5)
    self.connectmodel = ttk.Combobox(insertByConn, textvariable=self.connectedmodelvar) #, command = self.checkConnectivity
    self.connectmodel['values'] = ()
    self.connectmodel.bind('<<ComboboxSelected>>',self.checkConnectivity)
    self.connectmodel.place(x = 295, y = 5)

    #--------------- Connected face of inserting module -------
    label2 = Label(insertByConn, text='Face of New Module')
    label2.place(x = 75, y = 35)
    bard = Image.open("SmallSmores.png")
    bardejov = ImageTk.PhotoImage(bard)
    label3 = Label(insertByConn, image=bardejov)
    label3.image = bardejov
    label3.place(x=95, y=90)
    self.Back_face = Radiobutton(insertByConn, text='Back Face', variable=self.Node1, value=3)
    self.left_face = Radiobutton(insertByConn, text='Left Face', variable=self.Node1, value=1)
    self.right_face = Radiobutton(insertByConn, text='Right Face', variable=self.Node1, value=2)
    self.front_face = Radiobutton(insertByConn, text='Front Face', variable=self.Node1, value=0)
    self.front_face.select()
    self.Back_face.place(x= 125, y = 70,anchor = CENTER)
    self.front_face.place(x= 125, y = 200,anchor = CENTER)
    self.right_face.place(x= 45, y = 150,anchor = CENTER)
    self.left_face.place(x= 235, y = 150,anchor = CENTER)

    #--------------- Connected face of existing module -------
    label4 = Label(insertByConn, text='Face of Existing Module')
    label4.place(x = 365, y = 35)
    label5 = Label(insertByConn, image=bardejov)
    label5.image = bardejov
    label5.place(x=385, y=90)
    self.Back_face2 = Radiobutton(insertByConn, text='Back Face', variable=self.Node2, value=3)
    self.left_face2 = Radiobutton(insertByConn, text='Left Face', variable=self.Node2, value=1)
    self.right_face2 = Radiobutton(insertByConn, text='Right Face', variable=self.Node2, value=2)
    self.front_face2 = Radiobutton(insertByConn, text='Front Face', variable=self.Node2, value=0)
    self.front_face2.select()
    self.Back_face2.place(x= 415, y = 70,anchor = CENTER)
    self.front_face2.place(x= 415, y = 200,anchor = CENTER)
    self.right_face2.place(x= 325, y = 150,anchor = CENTER)
    self.left_face2.place(x= 525, y = 150,anchor = CENTER)

    #---------------- Position Entries -------------------------
    label6 = Label(insertByPos, text='X: ')
    label6.place(x = 30, y = 40)
    self.x_coor = Entry(insertByPos, textvariable=self.X_coor,width=10,state=NORMAL)
    self.x_coor.place(x = 30, y = 65)

    label7 = Label(insertByPos, text='Y: ')
    label7.place(x = 30, y = 90)
    self.y_coor = Entry(insertByPos, textvariable=self.Y_coor,width=10,state=NORMAL)
    self.y_coor.place(x = 30, y = 115)

    label8 = Label(insertByPos, text='Z: ')
    label8.place(x = 30, y = 140)
    self.Z_coor.set(0.05)
    self.z_coor = Entry(insertByPos, textvariable=self.Z_coor,width=10,state=NORMAL)
    self.z_coor.place(x = 30, y = 165)

    #--------------- Joint angle and orientation settings ------
    angleInfo = ttk.Labelframe(f1, text='Joint Angle and Model Orientation Settings', width = 765, height = 170)
    angleInfo.place(x = 5,y = 270)

    #--------------- Joint Angle Setting -----------------------
    label9 = Label(angleInfo, text='Joint Angle Bending ')
    label9.place(x = 40, y = 10)
    self.Joint3 = Scale(angleInfo, from_=-90, to=90, orient=HORIZONTAL,length = 150, resolution = 90)
    self.Joint3.place(x = 100, y = 55, anchor = CENTER)
    label10 = Label(angleInfo, text='Joint Angle Left Wheel ')
    label10.place(x = 210, y = 10)
    self.Joint1 = Scale(angleInfo, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 90)
    self.Joint1.place(x = 280, y = 55, anchor = CENTER)

    label11 = Label(angleInfo, text='Joint Angle Right Wheel ')
    label11.place(x = 30, y = 80)
    self.Joint2 = Scale(angleInfo, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 90)
    self.Joint2.place(x = 100, y = 125, anchor = CENTER)
    label12 = Label(angleInfo, text='Joint Angle Front Wheel ')
    label12.place(x = 210, y = 80)
    self.Joint0 = Scale(angleInfo, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 90)
    self.Joint0.place(x = 280, y = 125, anchor = CENTER)

    #--------------- Face Displacement ------------------------
    label13 = Label(angleInfo, text='Distance offset ')
    label13.place(x = 380, y = 10)
    doffset = Entry(angleInfo, textvariable=self.C_dis,width=10)
    doffset.place(x = 380, y = 40)

    label14 = Label(angleInfo, text='Angle offset')
    label14.place(x = 380, y = 80)
    # self.aoffset = Entry(angleInfo, textvariable=self.a_dis,width=10)
    self.aoffset = Scale(angleInfo, from_=-180, to=180, orient=HORIZONTAL,length = 100, resolution = 90, state = DISABLED)
    self.aoffset.place(x = 380, y = 105)

    #--------------- Model Orientation ------------------------
    label15 = Label(angleInfo, text='Model Orientation ')
    label15.place(x = 610, y = 10)

    label16 = Label(angleInfo, text='Row ')
    label16.place(x = 560, y = 45)
    self.row = Scale(angleInfo, from_=0, to=360, orient=HORIZONTAL,length = 100, resolution = 90)
    self.row.place(x = 660, y = 45, anchor = CENTER)
    label17 = Label(angleInfo, text='Pitch ')
    label17.place(x = 560, y = 85)
    self.pitch = Scale(angleInfo, from_=0, to=360, orient=HORIZONTAL,length = 100, resolution = 90)
    self.pitch.place(x = 660, y = 85, anchor = CENTER)
    label18 = Label(angleInfo, text='Yaw ')
    label18.place(x = 560, y = 125)
    self.yaw = Scale(angleInfo, from_=0, to=360, orient=HORIZONTAL,length = 100, resolution = 90)
    self.yaw.place(x = 660, y = 125, anchor = CENTER)

    #-------------- Select Model -----------------------------
    label19 = Label(f2, text='Select Model ')
    label19.place(x = 10, y = 10)
    self.modelselect = ttk.Combobox(f2, textvariable=self.modellist)
    self.modelselect['values'] = ()
    self.modelselect.bind('<<ComboboxSelected>>',self.CheckJointAngle)
    self.modelselect.place(x = 95, y = 10)

    #---------------- Joint Angle Setting -------------------
    lf = ttk.Labelframe(f2, text='Joint Angle Modification ', width = 350, height = 180)
    lf.place(x = 10, y = 50)
    label20 = Label(lf, text='Joint Angle Bending ')
    label20.place(x = 20, y = 10)
    self.Joint_3 = Scale(lf, from_=-90, to=90, orient=HORIZONTAL,length = 150, resolution = 5, command = self.ChangeJointAngle)
    self.Joint_3.bind('<ButtonRelease-1>',self.FindConnectable)
    self.Joint_3.place(x = 80, y = 50, anchor = CENTER)
    label21 = Label(lf, text='Joint Angle Left Wheel ')
    label21.place(x = 190, y = 10)
    self.Joint_1 = Scale(lf, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 5, command = self.ChangeJointAngle)
    self.Joint_1.bind('<ButtonRelease-1>',self.FindConnectable)
    self.Joint_1.place(x = 260, y = 50, anchor = CENTER)

    label22 = Label(lf, text='Joint Angle Right Wheel ')
    label22.place(x = 20, y = 80)
    self.Joint_2 = Scale(lf, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 5, command = self.ChangeJointAngle)
    self.Joint_2.bind('<ButtonRelease-1>',self.FindConnectable)
    self.Joint_2.place(x = 80, y = 120, anchor = CENTER)
    label23 = Label(lf, text='Joint Angle Front Wheel ')
    label23.place(x = 190, y = 80)
    self.Joint_0 = Scale(lf, from_=0, to=360, orient=HORIZONTAL,length = 150, resolution = 5, command = self.ChangeJointAngle)
    self.Joint_0.bind('<ButtonRelease-1>',self.FindConnectable)
    self.Joint_0.place(x = 260, y = 120, anchor = CENTER)

    #---------------- Delete Model ----------------------------
    self.DeleteButton = Button(f2, text="Delete", command = self.DeleteModule)
    self.DeleteButton["state"] = DISABLED
    self.DeleteButton["fg"]   = "red"
    self.DeleteButton.place(x = 5, y = window_height-Border_hieht-5, anchor = SW)

    #---------------- Add More Connections --------------------
    more_conn = ttk.Labelframe(f2, text='Add More Connections ', width = 350, height = 140)
    more_conn.place(x = 400, y = 50)

    label24 = Label(more_conn, text='Select Possible Connection ')
    label24.place(x = 10, y = 10)
    self.nodeselect = ttk.Combobox(more_conn, textvariable=self.adjacentnode, width = 38)
    self.nodeselect['values'] = ()
    self.nodeselect.bind('<<ComboboxSelected>>',self.EnableAddNewConnection)
    self.nodeselect.place(x = 10, y = 40)

    self.ConnectButton = Button(more_conn, text="Connect")
    self.ConnectButton["command"] = self.AddNewConnection
    self.ConnectButton.place(x = 260, y = 80)

    #---------------- Delete Whole Configuration -----------------
    del_conf = ttk.Labelframe(f2, text='Delete Entire Configuration ', width = 350, height = 50)
    del_conf.place(x = 400, y = 200)

    self.del_conf_confirm = Checkbutton(del_conf, text = "Please confirm", variable = self.del_confirm, onvalue = 1, offvalue = 0, command = self.ConfirmCheck, state = DISABLED)
    self.del_conf_confirm.place(x = 10, y = 5)

    self.DeleteConfigButton = Button(del_conf, text="delete", width='16', command = self.DeleteConfiguration, state = DISABLED)
    self.DeleteConfigButton.place(x = 180, y = 0)

    #---------------- Insert Model -------------------------------
    InsertButton = Button(f1, text="Insert")
    InsertButton["command"] = self.InsertModel
    InsertButton.place(x = 5, y = window_height-Border_hieht-5, anchor = SW)

    #---------------- Save Button --------------------------------
    # savepathlabel = Label(f1, text='Save path: ')
    # savepathlabel.place(x = window_width-Border_width-475, y = window_height-Border_hieht-7, anchor = SE)
    # savepathlabel2 = Label(f2, text='Save path: ')
    # savepathlabel2.place(x = window_width-Border_width-475, y = window_height-Border_hieht-7, anchor = SE)
    # self.savepathstr.set("~/.gazebo/models/SMORES7Stella/");
    # self.savepath1 = Entry(f1, textvariable=self.savepathstr, width = 35)
    # self.savepath1.place(x = window_width-Border_width-190, y = window_height-Border_hieht-7, anchor = SE)
    # self.savepath2 = Entry(f2, textvariable=self.savepathstr, width = 35)
    # self.savepath2.place(x = window_width-Border_width-190, y = window_height-Border_hieht-7, anchor = SE)
    self.saveButton = Button(f1, text="Save", command = self.WriteFile)
    self.saveButton.place(x = window_width-Border_width-125, y = window_height-Border_hieht-5, anchor = SE)
    self.saveButton2 = Button(f2, text="Save", command = self.WriteFile)
    self.saveButton2.place(x = window_width-Border_width-125, y = window_height-Border_hieht-5, anchor = SE)

    self.loadButton = Button(f1, text="Load", command = self.LoadFile)
    self.loadButton.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)
    self.loadButton2 = Button(f2, text="Load", command = self.LoadFile)
    self.loadButton2.place(x = window_width-Border_width-65, y = window_height-Border_hieht-5, anchor = SE)

  def LoadFile(self,*args):
    filename = tkFileDialog.askopenfilename(**self.file_opt)

    # open file on your own
    if filename:
      self.BuildConfigurationFromFile(filename)
      self.loadButton["state"] = DISABLED
      self.loadButton2["state"] = DISABLED

  def BuildConfigurationFromFile(self, filename):
    # Delete the exiting configuration first
    self.DeleteConfiguration()
    # Build new configurations
    self.tree = ET.parse(filename)
    root = self.tree.getroot()
    modules = root.find("modules")
    for eachmodule in modules.findall('module') :
      model_name = eachmodule.find('name').text
      positionstr = eachmodule.find('position').text
      # print "position: ",self.StringToTuple(positionstr)
      module_position = self.StringToTuple(positionstr)
      jointanglestr = eachmodule.find('joints').text
      try:
        module_path = eachmodule.find('path').text
      except Exception, e:
        module_path = self.DEFAULT_PATH
      print "module path: ",module_path
      # print "joint angles: ",self.StringToTuple(jointanglestr)
      module_jointangle = self.StringToTuple(jointanglestr)
      if len(module_position) == 6:
          new_module = Module(model_name,module_position,module_jointangle,module_path)
          new_module.rotation_matrix = kinematics.rotz(module_position[5])* \
                                       kinematics.roty(module_position[4])* \
                                       kinematics.rotx(module_position[3])
      elif len(module_position) == 7:
        new_module = Module(model_name,module_position,module_jointangle,module_path,True)
        new_module.rotation_matrix = kinematics.quatToRot(module_position[3:])
      self.ModuleList.append(new_module)
      if self.ServerConnected == 1:
        self.PublishMessage(self.ModuleList[-1])

    # Build Connection Information
    connections = root.find("connections")
    for eachconnection in connections.findall('connection') :
      new_connection = Connection(eachconnection.find('module1').text,eachconnection.find('module2').text,int(eachconnection.find('node1').text),int(eachconnection.find('node2').text),float(eachconnection.find('distance').text),float(eachconnection.find('angle').text))
      self.ConnectionList.append(new_connection)
      self.GetModelByname(eachconnection.find('module1').text).connection(int(eachconnection.find('node1').text),self.ConnectionList[-1])
      self.GetModelByname(eachconnection.find('module2').text).connection(int(eachconnection.find('node2').text),self.ConnectionList[-1])
    print "Total connections: ",len(self.ConnectionList)

    self.updateModuleList()
    self.nameIncrement()
    self.Rel_pos.select()
    self.DisableXYZInput()
    self.SaveEnable()
    self.connectmodel.set('')

  def StringToTuple(self, anglestring):
    jointangles = [float(x) for x in anglestring.split()]
    return tuple(jointangles)

  def GetModelByname(self,name):
    models = [eachmodel for eachmodel in self.ModuleList if eachmodel.ModelName == name]
    return models[0]

  def SelectModelType(self):
    fileoptions = {}
    fileoptions['filetypes'] = [('Model type files', '.modeltype'),('all files', '*')]
    fileoptions['initialdir'] = '~/'
    fileoptions["defaultextension"] = ".modeltype"
    fileoptions['parent'] = self.parent
    fileoptions['title'] = 'Select Module Type'
    filename = tkFileDialog.askopenfilename(**fileoptions)
    if filename:
      self.tree = ET.parse(filename)
      root = self.tree.getroot()
      self.current_model_type = root.find("type").text
      self.current_suggest_name = root.find("suggested_name").text
      self.current_modeule_path = root.find("file").text
      self.type_name['text'] = self.current_model_type
      self.nameIncrement()

  def SetTypeDefault(self):
    self.current_model_type = self.DEFAULT_TYPE
    self.current_suggest_name = "Module"
    self.current_modeule_path = self.DEFAULT_PATH
    self.type_name['text'] = self.current_model_type
    self.nameIncrement()

  def InsertModel(self,*args):
    print "frob called with {} arguments".format(len(args))
    insertedModel = False
    if self.InsertMethod.get() == "Position" :
      print "Method is position"
      module_position = (self.X_coor.get(),self.Y_coor.get(),self.Z_coor.get(),degree2rad(self.row.get()),degree2rad(self.pitch.get()),degree2rad(self.yaw.get()))
      print "Position tuple is ",module_position
      module_jointangle = (degree2rad(self.Joint0.get()),degree2rad(self.Joint1.get()),degree2rad(self.Joint2.get()),degree2rad(self.Joint3.get()))
      print "Joint angle tuple is ",module_jointangle
      new_module = Module(self.modelname.get(),module_position,module_jointangle,self.current_modeule_path)
      # Assign rotation matrix to new_module based on euler angles:
      new_module.rotation_matrix = kinematics.rotz(module_position[5])* \
                                   kinematics.roty(module_position[4])* \
                                   kinematics.rotx(module_position[3])
      self.ModuleList.append(new_module)
      if self.ServerConnected == 1:
        self.PublishMessage(self.ModuleList[-1])
      insertedModel = True
    else:
      print "Method is Connection"
      theOtherModule = self.findModule(self.connectedmodelvar.get())
      if theOtherModule:
        module_jointangle = (degree2rad(self.Joint0.get()),degree2rad(self.Joint1.get()),degree2rad(self.Joint2.get()),degree2rad(self.Joint3.get()))
        #-- Get module position and orientation.
        parent_face = self.Node2.get()
        new_module_face = self.Node1.get()
        (module_position, rotation_matrix, quaternion) = kinematics.get_new_position(theOtherModule, module_jointangle, parent_face, new_module_face)
        print 'XYZ: ' + str(module_position[0:3])
        print 'RPY: ' + str(module_position[3:6])
        print 'Quat: ' + str(quaternion)
        # --
        new_module = Module(self.modelname.get(),tuple(list(module_position[0:3])+list(quaternion)),module_jointangle, self.current_modeule_path,True)
        # Add rotation matrix to new_module (necessary for kinematics)
        new_module.rotation_matrix = rotation_matrix
        self.ModuleList.append(new_module)
        print "Connected module name",self.connectedmodelvar.get()
        new_connection = Connection(self.modelname.get(),self.connectedmodelvar.get(),self.Node1.get(),self.Node2.get(),self.C_dis.get(),self.aoffset.get())
        self.ConnectionList.append(new_connection)
        self.ModuleList[-1].connection(self.Node1.get(),self.ConnectionList[-1])
        theOtherModule.connection(self.Node2.get(),self.ConnectionList[-1])
        if self.ServerConnected == 1:
          self.PublishMessage(self.ModuleList[-1])
        insertedModel = True
    if insertedModel:
      self.updateModuleList()
      self.nameIncrement()
      self.Rel_pos.select()
      self.DisableXYZInput()
      self.SaveEnable()
      print "Combox value", self.connectmodel.get()
      if self.connectmodel.get() != '':
        self.checkConnectivity()

  def EnableXYZInput(self,*args):
    self.x_coor["state"] = NORMAL
    self.y_coor["state"] = NORMAL
    self.z_coor["state"] = NORMAL
    self.aoffset.set(0)
    self.aoffset["state"] = DISABLED
    self.DisableInsertByConnection()

  def DisableXYZInput(self,*args):
    self.x_coor["state"] = DISABLED
    self.y_coor["state"] = DISABLED
    self.z_coor["state"] = DISABLED
    self.aoffset["state"] = NORMAL
    self.aoffset.set(0)
    self.EnableInsertByConnection()

  def updateModuleList(self):
    a_model_list = []
    for eachmodel in self.ModuleList:
      a_model_list.append(eachmodel.ModelName)
    self.ModuleNameList = a_model_list
    self.modelselect['values'] = tuple(a_model_list)
    self.connectmodel['values'] = tuple(a_model_list)

  def nameIncrement(self):
    incr_num = 0
    while 1:
      module_name_tmp = self.current_suggest_name+'_'+str(incr_num)
      if (module_name_tmp in self.ModuleNameList) or \
          (module_name_tmp in self.nameTrash):
        incr_num += 1
      else:
        self.modelname.set(module_name_tmp)
        break

  # Put the position calculation code here
  def CalculatePosition(self):
    return (0.0,0.0,0.0,0.0,0.0,0.0)

  def findModule(self,modelname):
    for eachmodule in self.ModuleList:
      if eachmodule.ModelName == modelname:
        return eachmodule
    return False

  def checkConnectivity(self,*args):
    themodule = self.findModule(self.connectedmodelvar.get())
    if themodule:
      if len(themodule.nodes[3]) != 0:
        self.Back_face2["state"] = DISABLED
      else:
        self.Back_face2["state"] = NORMAL
        self.Back_face2.select()
      if len(themodule.nodes[1]) != 0:
        self.left_face2["state"] = DISABLED
      else:
        self.left_face2["state"] = NORMAL
        self.left_face2.select()
      if len(themodule.nodes[2]) != 0:
        self.right_face2["state"] = DISABLED
      else:
        self.right_face2["state"] = NORMAL
        self.right_face2.select()
      if len(themodule.nodes[0]) != 0:
        self.front_face2["state"] = DISABLED
      else:
        self.front_face2["state"] = NORMAL
        self.front_face2.select()

  def PublishMessage(self,amodule):
    newmessage = ConfigMessage()
    newmessage.ModelName = amodule.ModelName
    if amodule.Quaternion:
        for i in xrange(7):
          newmessage.ModelPosition.append(amodule.Position[i])
        newmessage.QuaternionPos = True
    else:
        for i in xrange(6):
          newmessage.ModelPosition.append(amodule.Position[i])
    print "Coor info",newmessage.ModelPosition
    for i in xrange(4):
      newmessage.JointAngles.append(amodule.JointAngle[i])
    newmessage.ModelPath = amodule.Path
    # self.communicator.publish(newmessage)
    self.configPub.Publish(newmessage)
    # ++++++++ These Lines for pygazebo +++++++++++++
    # self.newconnection.sendData(newmessage)
    # self.publisher.publish(newmessage)
    # eventlet.sleep(1.0)
    print "Information published"

  def CloseWindow(self):
    if self.initflag==0 or self.initflag==2:
      # self.communicator.stop()
      self.communicator.Close()
    if self.initflag==0:
      self.rungzserver.terminate()
      self.rungzclient.terminate()
      call(["pkill", "gzserver"])
      call(["pkill", "gzclient"])
    self.quit()

  def CheckJointAngle(self,*args):
    modelname = self.modellist.get()
    modelobj = self.findModule(modelname)
    if modelobj.ModelName:
      self.Joint_3.set(int(modelobj.JointAngle[3]/PI*180))
      self.Joint_2.set(int(modelobj.JointAngle[2]/PI*180))
      self.Joint_1.set(int(modelobj.JointAngle[1]/PI*180))
      self.Joint_0.set(int(modelobj.JointAngle[0]/PI*180))
    # Lock the connected joint
      if len(modelobj.nodes[0]) > 0:
        self.Joint_0["state"] = DISABLED
      else:
        self.Joint_0["state"] = NORMAL
      if len(modelobj.nodes[1]) > 0:
        self.Joint_1["state"] = DISABLED
      else:
        self.Joint_1["state"] = NORMAL
      if len(modelobj.nodes[2]) > 0:
        self.Joint_2["state"] = DISABLED
      else:
        self.Joint_2["state"] = NORMAL
      if len(modelobj.nodes[3]) > 0:
        self.Joint_3["state"] = DISABLED
      else:
        self.Joint_3["state"] = NORMAL
    self.FindConnectable()
    self.DeleteButtonEnable()

  def ChangeJointAngle(self,*args):
    print "Angle Changed"
    modelname = self.modellist.get()
    modelobj = self.findModule(modelname)
    if modelobj:
      modelobj.JointAngle = (degree2rad(self.Joint_0.get()),degree2rad(self.Joint_1.get()),degree2rad(self.Joint_2.get()),degree2rad(self.Joint_3.get()))
      print "joint angles ",modelobj.JointAngle
      if self.ServerConnected == 1:
        self.PublishMessage(modelobj)
      self.SaveEnable()

  def WriteFile(self):
    gait_file_opt = {}
    gait_file_opt['filetypes'] = [('configuration file', '.conf'),('XML file', '.xml'),('text files', '.txt'),('all files', '*')]
    gait_file_opt["defaultextension"] = ".conf"
    gait_file_opt['initialdir'] = '~/.gazebo/models/'
    gait_file_opt['parent'] = self.parent
    gait_file_opt['title'] = 'Save Configuration File'
    f = tkFileDialog.asksaveasfile(mode='w', **gait_file_opt)
    if f is None: # asksaveasfile return `None` if dialog closed with "cancel".
        return
    lines = ['<?xml version="1.0" encoding="UTF-8"?>\n']
    lines.append('<configuration>\n')
    lines.append('<modules>\n')
    for eachmodule in self.ModuleList:
      lines.append('\t<module>\n')
      lines.append('\t\t<name>'+eachmodule.ModelName+'</name>\n')
      if len(eachmodule.Position) == 6:
        lines.append('\t\t<position>'+str(eachmodule.Position[0])+' '+ \
            str(eachmodule.Position[1])+' '+str(eachmodule.Position[2])+' '+ \
            str(eachmodule.Position[3])+' '+str(eachmodule.Position[4])+' '+ \
            str(eachmodule.Position[5])+'</position>\n')
      elif len(eachmodule.Position) == 7:
        lines.append('\t\t<position>'+str(eachmodule.Position[0])+' '+ \
            str(eachmodule.Position[1])+' '+str(eachmodule.Position[2])+' '+ \
            str(eachmodule.Position[3])+' '+str(eachmodule.Position[4])+' '+ \
            str(eachmodule.Position[5])+' '+str(eachmodule.Position[6])+ \
            '</position>\n')
      lines.append('\t\t<joints>'+str(eachmodule.JointAngle[0])+' '+str(eachmodule.JointAngle[1])+' '+str(eachmodule.JointAngle[2])+' '+str(eachmodule.JointAngle[3])+'</joints>\n')
      lines.append('\t\t<path>'+eachmodule.Path+'</path>\n')
      lines.append('\t</module>\n')
    lines.append('</modules>\n\n')
    lines.append('<connections>\n')
    for eachconnection in self.ConnectionList:
      lines.append('\t<connection>\n')
      lines.append('\t\t<module1>'+eachconnection.Module1+'</module1>\n')
      lines.append('\t\t<module2>'+eachconnection.Module2+'</module2>\n')
      lines.append('\t\t<node1>'+str(eachconnection.Node1)+'</node1>\n')
      lines.append('\t\t<node2>'+str(eachconnection.Node2)+'</node2>\n')
      lines.append('\t\t<distance>'+str(eachconnection.Distance)+'</distance>\n')
      lines.append('\t\t<angle>'+str(eachconnection.Angle)+'</angle>\n')
      lines.append('\t</connection>\n')
    lines.append('</connections>\n')
    lines.append('</configuration>')
    f.writelines(lines)
    f.close()
    self.saveButton["state"] = DISABLED
    self.saveButton2["state"] = DISABLED

  def SaveEnable(self):
    self.saveButton["state"] = NORMAL
    self.saveButton2["state"] = NORMAL

  def FindConnectable(self,*args):
    modelname = self.modellist.get()
    modelobj = self.findModule(modelname)
    closeModulesCandidate = []
    for eachmodel in self.ModuleList:
      if CloseEnough(modelobj.Position,eachmodel.Position):
        closeModulesCandidate.append(eachmodel)
    connectedModel = []
    for i in xrange(4):
      if len(modelobj.nodes[i])>0:
        connectedModel.append(modelobj.nodes[i].Module1)
        connectedModel.append(modelobj.nodes[i].Module2)
    closeModules = [eachmodel for eachmodel in closeModulesCandidate if not eachmodel.ModelName in connectedModel]    
    connectableList = []
    self.connectableRealList = []
    for eachmodel in closeModules:
      connectNode = Connectable(modelobj,modelobj.JointAngle,eachmodel,eachmodel.JointAngle)
      if connectNode :
        connectableList.append(self.GetNodenameByNodeNumber(connectNode[0])+":"+eachmodel.ModelName+"-"+self.GetNodenameByNodeNumber(connectNode[1]))
        self.connectableRealList.append([modelobj,eachmodel,connectNode])
    self.nodeselect['values'] = tuple(connectableList)

  def AddNewConnection(self):
    indx = self.nodeselect['values'].index(self.adjacentnode.get())
    new_connection = Connection(self.connectableRealList[indx][0].ModelName,self.connectableRealList[indx][1].ModelName,self.connectableRealList[indx][2][0],self.connectableRealList[indx][2][1],0,0)
    self.ConnectionList.append(new_connection)
    self.connectableRealList[indx][0].connection(self.connectableRealList[indx][2][0],self.ConnectionList[-1])
    self.connectableRealList[indx][1].connection(self.connectableRealList[indx][2][1],self.ConnectionList[-1])
    self.adjacentnode.set("")
    self.ConnectButton["state"] = DISABLED
    self.FindConnectable()

  def EnableAddNewConnection(self,*args):
    self.ConnectButton["state"] = NORMAL

  def DeleteModule(self):
    newmessage = ConfigMessage()
    newmessage.ModelName = self.modellist.get()
    for idx,eachmodel in enumerate(self.ModuleList):
      if eachmodel.ModelName == newmessage.ModelName:
        for eachnode in xrange(4):
          if len(eachmodel.nodes[eachnode])>0:
            nodeidx = self.ConnectionList.index(eachmodel.nodes[eachnode])
            acconection = eachmodel.nodes[eachnode]
            self.findModule(acconection.Module1).disconnect(acconection.Node1)
            self.findModule(acconection.Module2).disconnect(acconection.Node2)
            del self.ConnectionList[nodeidx]
        del self.ModuleList[idx]
        break
    for i in xrange(6):
      newmessage.ModelPosition.append(0)
    for i in xrange(4):
      newmessage.JointAngles.append(0)
    newmessage.DeleteFlag = True
    # self.communicator.publish(newmessage)
    self.configPub.Publish(newmessage)
    os.system("gz model -d -m "+newmessage.ModelName)
    self.nameTrash.append(newmessage.ModelName)
    # delete_module = Popen(['gz', 'model','-d','-m',newmessage.ModelName], stdout=PIPE)
    # time.sleep(0.5)
    # delete_module.kill()
    print "Module deleted"
    self.updateModuleList()
    if not (self.connectedmodelvar.get() in self.ModuleNameList):
      self.connectedmodelvar.set('')
    self.checkConnectivity()
    self.modellist.set('')
    self.DeleteButtonDisable()

  def DeleteConfiguration(self):
    print "Model names: ",[x.ModelName for x in self.ModuleList]
    for eachmodel in self.ModuleList:
      newmessage = ConfigMessage()
      newmessage.ModelName = eachmodel.ModelName
      for i in xrange(6):
        newmessage.ModelPosition.append(0)
      for i in xrange(4):
        newmessage.JointAngles.append(0)
      newmessage.DeleteFlag = True
      # self.communicator.publish(newmessage)
      self.configPub.Publish(newmessage)
      call(['gzfactory','delete','-m',eachmodel.ModelName])

    self.ModuleList = []
    self.ConnectionList = []
    self.updateModuleList()
    self.modellist.set('')
    self.DeleteButtonDisable()
    self.del_conf_confirm.deselect()
    self.DeleteConfigButton["state"] = DISABLED

  def DeleteButtonDisable(self):
    self.DeleteButton["state"] = DISABLED

  def DeleteButtonEnable(self):
    self.DeleteButton["state"] = NORMAL

  def GetNodenameByNodeNumber(self,node):
    if node == 0:
      return "Front"
    if node == 1:
      return "Left"
    if node == 2:
      return "Right"
    if node == 3:
      return "Back"

  def ConfirmCheck(self):
    if self.del_confirm.get() == 1:
      self.DeleteConfigButton["state"] = NORMAL
    else:
      self.DeleteConfigButton["state"] = DISABLED

  def DisableInsertByConnection(self):
    self.connectmodel["state"] = DISABLED
    self.Back_face["state"] = DISABLED
    self.left_face["state"] = DISABLED
    self.right_face["state"] = DISABLED
    self.front_face["state"] = DISABLED
    self.Back_face2["state"] = DISABLED
    self.left_face2["state"] = DISABLED
    self.right_face2["state"] = DISABLED
    self.front_face2["state"] = DISABLED

  def EnableInsertByConnection(self):
    self.connectmodel["state"] = NORMAL
    self.Back_face["state"] = NORMAL
    self.left_face["state"] = NORMAL
    self.right_face["state"] = NORMAL
    self.front_face["state"] = NORMAL
    self.Back_face2["state"] = NORMAL
    self.left_face2["state"] = NORMAL
    self.right_face2["state"] = NORMAL
    self.front_face2["state"] = NORMAL

  def subcallback(self, msg):
    print "in the sub call back"

def degree2rad(angle):
  return angle/180.0*PI
                     
def main(flag):
  
  root = Tk()
  root.geometry(str(window_width)+"x"+str(window_height))
  app = ConfigEditor(root,flag)
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

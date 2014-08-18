## @package Module Module object that used by configuration editor

## This is the class that stores all the information about a module
class Module:
  ## Constructor
  # @param self Object pointer
  # @param modelname Model name string
  # @param position Tuple of the initial position of the module
  # @param jointangle List of the joint angles of the module
  # @param path Path of the model sdf file
  # @quapos Whether the orientation of the module stored in quaternion 
  #         True: stored in quaternion; False: stored in Eular angles; 
  #         Default: False
  def __init__(self, modelname,position,jointangle, path, quapos = False):
    ## Module name strong
    self.ModelName = modelname
    try:
      ## Module position tuple
      self.Position = position
    except ValueError:
      pass
    try:
      ## Module joint angle list
      self.JointAngle = jointangle
    except ValueError:
      pass
    ## Module node dictionary
    # Keys are the node indecies and values are pointer to the Connection object
    self.nodes = {0:[],1:[],2:[],3:[]}
    ## Path of the module sdf file
    self.Path = path
    ## Flag of whether the orientation of the module stored in quaternion format
    self.Quaternion = quapos
  ## Connect on one node
  # @param self Object pointer
  # @param node A node index
  # @param connection A Connection object
  def connection(self,node,connection):
  	self.nodes[node] = connection
  ## Disconnect on one node
  # @param self Object pointer
  # @param node A node index
  def disconnect(self,node):
  	self.nodes[node] = []
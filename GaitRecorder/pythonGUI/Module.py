## @package Module Module object that used by gait recorder

## This is the class that stores all the information about a module
class Module:
  ## Constructor
  # @param self Object pointer
  # @param modelname Model name string
  # @param jointangle List of the joint angles of the module
  # @param position Tuple of the initial position of the module
  def __init__(self, modelname,jointangle,position):
    ## Module name strong
    self.ModelName = modelname
    try:
      ## Module position tuple
      self.JointAngle = jointangle
    except ValueError:
      pass
    try:
      ## Module joint angle tuple
      self.Position = position
    except ValueError:
      pass
    ## Speed of the joints
    # TODO: haven't really implement this part
    self.Speeds = [0,0,0]
    ## Module node dictionary
    # Keys are the node indecies and values are pointer to the Connection object
    self.nodes = {0:[],1:[],2:[],3:[]}
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
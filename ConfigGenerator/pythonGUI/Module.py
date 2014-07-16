class Module:
  def __init__(self, modelname,position,jointangle, quapos = False):
    self.ModelName = modelname
    try:
      self.Position = position
    except ValueError:
      pass
    try:
      self.JointAngle = jointangle
    except ValueError:
      pass
    self.nodes = {0:[],1:[],2:[],3:[]}
    self.Quaternion = quapos

  def connection(self,node,connection):
  	self.nodes[node] = connection

  def disconnect(self,node):
  	self.nodes[node] = []
class Module:
  def __init__(self, modelname,position,jointangle):
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

  def connection(self,node,connection):
  	self.nodes[node] = connection

  def disconnect(self,node):
  	self.nodes[node] = []
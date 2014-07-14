class Module:
  def __init__(self, modelname,jointangle,position):
    self.ModelName = modelname
    try:
      self.JointAngle = jointangle
    except ValueError:
      pass
    try:
      self.Position = position
    except ValueError:
      pass
    self.Speeds = [0,0,0]
    self.nodes = {0:[],1:[],2:[],3:[]}

  def connection(self,node,connection):
    self.nodes[node] = connection

  def disconnect(self,node):
    self.nodes[node] = []
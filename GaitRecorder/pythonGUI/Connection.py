class Connection:
  def __init__(self,model1,model2,node1,node2,distance=0,angle=0):
    self.Module1 = model1   # A Module object
    self.Module2 = model2   # A Module object
    self.Node1 = node1
    self.Node2 = node2
    self.Distance = distance
    self.Angle = angle
  def __len__(self):
    return 1

  def GetConnectModuleAndNode(self,module_name):
    if module_name == self.Module1.ModelName:
      return (self.Module2.ModelName, node2)
    elif module_name == self.Module2.ModelName:
      return (self.Module1.ModelName, node1)
    else:
      return (False, False)
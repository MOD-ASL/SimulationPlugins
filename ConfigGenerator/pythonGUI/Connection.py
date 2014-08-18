## @package Connection Connection object that used by Configuration editor

## The class that has all the information about a connection between two modules
class Connection:
  ## Constructor
  # @param self Object pointer
  # @param model1 A Module object of one connected module
  # @param model2 A module object of the other connected module
  # @param node1 Connected node index of module1
  # @param node2 Connected node index of module2
  # @param distance Connection distance offset, default: 0
  # @param angle Connection angle offset, default: 0
	def __init__(self,model1,model2,node1,node2,distance=0,angle=0):
    # Name string of module 1
		self.Module1 = model1
    # Name string of module 2
		self.Module2 = model2
    # Node index of the connected face of module 1
		self.Node1 = node1
    # Node index of the connected face of module 2
		self.Node2 = node2
    # Distance offset
		self.Distance = distance
    # Angle offset
		self.Angle = angle
  ## len definition
  # @param self Object pointer
	def __len__(self):
		return 1
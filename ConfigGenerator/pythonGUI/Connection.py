class Connection:
	def __init__(self,model1,model2,node1,node2,distance=0,angle=0):
		self.Module1 = model1
		self.Module2 = model2
		self.Node1 = node1
		self.Node2 = node2
		self.Distance = distance
		self.Angle = angle
	def __len__(self):
		return 1
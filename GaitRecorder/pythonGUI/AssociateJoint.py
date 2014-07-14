class AssociateJoint:
	def __init__(self, module, node, corr, ratio):
		self.ModuleName = module 	# name string
		self.Node = node
		self.Correlation = corr 	# bool value
		self.Ratio = ratio

	def ToString(self):
		return self.ModuleName+"::"+self.NodeToString(self.Node)+"::"+	\
				self.CorrelationToStr(self.Correlation)+"::"+str(self.Ratio)

	def NodeToString(self, node):
		if node == 0:
			return "Front Wheel"
		if node == 1:
			return "Lft Wheel"
		if node == 2:
			return "Rgt Wheel"
		if node == 3:
			return "Central Bending"

	def CorrelationToStr(self,corr):
		if corr:
			return "+"
		else:
			return "-"
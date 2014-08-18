## @package AssociateJoint Association joint that used by gait recorder

## The class that has all the information about associations
class AssociateJoint:
	## Constructor
	# @param self Object pointer
	# @param module Module name string
	# @param node Node index
	# @param corr Bool, correaltion: True for positive; False for negtive
	# @param ratio Correlation ratio
	def __init__(self, module, node, corr, ratio):
		## Module name string
		self.ModuleName = module 	# name string
		## Node index
		self.Node = node
		## Correlation boolean value
		self.Correlation = corr 	# bool value
		## Correlation ratio
		self.Ratio = ratio
	## Current object to string
	# @param self Object pointer
	def ToString(self):
		return self.ModuleName+"::"+self.NodeToString(self.Node)+"::"+	\
				self.CorrelationToStr(self.Correlation)+"::"+str(self.Ratio)
	## Find node string name given node index
	# @param self Object pointer
	# @param node Integer, node indez
	def NodeToString(self, node):
		if node == 0:
			return "Front Wheel"
		if node == 1:
			return "Lft Wheel"
		if node == 2:
			return "Rgt Wheel"
		if node == 3:
			return "Central Bending"
	## Correlation boolean value to string
	# @param self Object pointer
	# @param corr Correlation boolean
	def CorrelationToStr(self,corr):
		if corr:
			return "+"
		else:
			return "-"
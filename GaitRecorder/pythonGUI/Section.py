## @package Section Section object that used by Gait Recorder

from GaitEntry import *
## A class that stores a group of gaits which can finish a small task
class Section:
	## Constructor
	# @param self Object pointer
	# @param frame_id Section identity string
	def __init__(self, frame_id):
		## Section identity string
		self.FrameID = frame_id
		## List of the GaitEntry in the current section
		self.GaitEntries = []
		## Liast of inital joint states
		self.InitialPosition = []
		## List of modules in this section
		self.ModulesInThisFrame = []
		## List of the gait strings in this section
		self.GaitStrListOfModule = []
		## List of GaitEntry object in this section
		self.GaitObjListOfModule = []
	## Keep a record of all the joints angles
	# @param self Object pointer
	# @param module_list A list of Module object
	def RecordCurrentPosition(self,module_list):
		for eachmodule in module_list:
			self.InitialPosition.append(eachmodule.JointAngle)
	## Add a gait to current section
	# @param self Object pointer
	# @param a_gait A GaitEntry object
	def AddGaitToSection(self, a_gait):
		self.GaitEntries.append(a_gait)
	## Add a module in the current section module list
	# @param self Object pointer
	# @param a_gait An GaitEntry object
	def AddNewChangedModuleToFrame(self, a_gait):
		# self.GaitEntries.append(a_gait)
		if not a_gait.ModuleName in self.ModulesInThisFrame:
			self.ModulesInThisFrame.append(a_gait.ModuleName)
	## Clear current section
	# @param self Object pointer
	def EmptyTheContainer(self):
		self.ModulesInThisFrame = []
		self.GaitStrListOfModule = []
	## Add empty GaitEntry before the given gait to align all the gait that executes at the same time
	# @param self Object pointer
	# @param a_gait  A GaitEntry object
	# @param list_idx Index of the position of the given gait
	def AddGaitToModule(self,a_gait,list_idx = -1):
		if list_idx == -1:
			if a_gait.dependency_id != "":
				idx = self.FindLastPisitionOfACondition(a_gait.dependency_id)
				occupy_gait = GaitEntry("",[0.0,0.0,0.0,0.0],0)
				self.GaitObjListOfModule.append([occupy_gait]*(idx+1)+[a_gait])
				self.GaitStrListOfModule.append([""]*(idx+1)+[a_gait.GaitToStr()])
				# print "GaitStrListOfModule: ",self.GaitStrListOfModule
			else:
				self.GaitObjListOfModule.append([a_gait])
				self.GaitStrListOfModule.append([a_gait.GaitToStr()])
		else:
			if a_gait.dependency_id != "":
				idx = self.FindLastPisitionOfACondition(a_gait.dependency_id)
				occupy_gait = GaitEntry("",[0.0,0.0,0.0,0.0],0)
				ext_len = len(self.GaitStrListOfModule[list_idx])
				self.GaitObjListOfModule[list_idx] += [occupy_gait]*(idx+1-ext_len)+[a_gait]
				self.GaitStrListOfModule[list_idx] += [""]*(idx+1-ext_len)+[a_gait.GaitToStr()]
			else:
				self.GaitObjListOfModule[list_idx].append(a_gait)
				self.GaitStrListOfModule[list_idx].append(a_gait.GaitToStr())
	## Find the last appearance of a condition string
	# @param self Object pointer
	# @param a_condition A condition string
	# @return The position of the last GaitEntry object
	def FindLastPisitionOfACondition(self,a_condition):
		indx = -1
		for each_list in self.GaitObjListOfModule:
			for idx, each_entry in enumerate(each_list):
				if each_entry.condition_id == a_condition:
					# print "Find a condition match"
					if idx >indx:
						indx = idx
		return indx
from GaitEntry import *

class Section:
	def __init__(self, frame_id):
		self.FrameID = frame_id
		self.GaitEntries = []
		self.InitialPosition = []
		self.ModulesInThisFrame = []
		self.GaitStrListOfModule = []
		self.GaitObjListOfModule = []

	def RecordCurrentPosition(self,module_list):
		for eachmodule in module_list:
			self.InitialPosition.append(eachmodule.JointAngle)

	def AddGaitToSection(self, a_gait):
		self.GaitEntries.append(a_gait)

	def AddNewChangedModuleToFrame(self, a_gait):
		# self.GaitEntries.append(a_gait)
		if not a_gait.ModuleName in self.ModulesInThisFrame:
			self.ModulesInThisFrame.append(a_gait.ModuleName)

	def EmptyTheContainer(self):
		self.ModulesInThisFrame = []
		self.GaitStrListOfModule = []

	def AddGaitToModule(self,a_gait,list_idx = -1):
		if list_idx == -1:
			if a_gait.dependency_id != "":
				idx = self.FindLastPisitionOfACondition(a_gait.dependency_id)
				occupy_gait = GaitEntry("",[0.0,0.0,0.0,0.0],0)
				self.GaitObjListOfModule.append([occupy_gait]*(idx+1)+[a_gait])
				self.GaitStrListOfModule.append([""]*(idx+1)+[a_gait.GaitToStr()])
				print "GaitStrListOfModule: ",self.GaitStrListOfModule
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

	def FindLastPisitionOfACondition(self,a_condition):
		indx = -1
		for each_list in self.GaitObjListOfModule:
			for idx, each_entry in enumerate(each_list):
				if each_entry.condition_id == a_condition:
					print "Find a condition match"
					if idx >indx:
						indx = idx
		return indx
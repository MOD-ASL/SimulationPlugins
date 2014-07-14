class GaitEntry:
  def __init__(self,module_id,jointangles,timer,dependency = "",condition = "",special = False, flags = [0,0,0,0], extra_info = ""):
    self.ModuleName = module_id
    self.Joints = jointangles
    self.condition_id = condition
    self.dependency_id = dependency
    self.Timer = timer  # 0 means no timer at all
    self.AngleFlags = flags   # 0 for position, 1 for speed, 2 for torque
    self.SpecialEntry = special
    self.ExtraInfo = extra_info

  def GaitToStr(self):
    if self.SpecialEntry :
      gaitstr = self.ExtraInfo
      return gaitstr
    else:
      gaitstr = ""
      gaitstr += self.ModuleName+" "
      for i in xrange(4):
        if self.AngleFlags[i] == 0:
          gaitstr+= "p"
        elif self.AngleFlags[i] == 1:
          gaitstr+= "s"
        elif self.AngleFlags[i] == 2:
          gaitstr+= "t"        
        gaitstr+= str(self.Joints[i])+" "
      if self.Timer > 0:
        gaitstr+= "["+str(self.Timer)+"] "
      if len(self.condition_id) > 0 :
        gaitstr+= "{"+self.condition_id+"} "
      if len(self.dependency_id) > 0 :
        gaitstr+= "("+self.dependency_id+") "
      gaitstr+=";"
      return gaitstr

  def AddExtraInfo(self,info_str):
    self.ExtraInfo = info_str
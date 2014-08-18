## @package GaitEntry GaitEntry object that used by gait recorder

## The class that stores a gait command
class GaitEntry:
  ## Constructor
  # @param self Object pointer
  # @param module_id String of module name
  # @param jointangles List of joint angles
  # @param timer Timer for time based gait
  # @param dependency Gait trigger condition string 
  # @param condition Condition string that triggers other commands
  # @param special Whether this is a special command
  # @param flags Joint setting flags: 0 for position, 1 for speed, 2 for torque, 3 for ignore, 4 for connection, 5 for disconnection
  def __init__(self,module_id,jointangles,timer,dependency = "",condition = "",special = False, flags = [0,0,0,0], extra_info = ""):
    ## Module name string
    self.ModuleName = module_id
    ## Tuple of joint angles
    self.Joints = jointangles
    ## Condition string
    self.condition_id = condition
    ## Dependency string
    self.dependency_id = dependency
    ## Timer for time based gaits
    self.Timer = timer  # 0 means no timer at all
    ## Joint setting flags
    self.AngleFlags = flags   # 0 for position, 1 for speed, 2 for torque
    ## Special command flag
    self.SpecialEntry = special
    ## Extra information string
    self.ExtraInfo = extra_info
  ## Current gait to gait string
  # @param self Object pointer 
  def GaitToStr(self):
    if self.SpecialEntry :
      gaitstr = self.ExtraInfo+";"
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
        elif self.AngleFlags[i] == 3:
          gaitstr+= "i"
        elif self.AngleFlags[i] == 4:
          gaitstr+= "c"
        elif self.AngleFlags[i] == 5:
          gaitstr+= "d"
        if self.AngleFlags[i] < 3 :
          gaitstr+= str(self.Joints[i])+" "
        else:
          gaitstr+= " "
      if self.Timer > 0:
        gaitstr+= "["+str(self.Timer)+"] "
      if len(self.condition_id) > 0 :
        gaitstr+= "{"+self.condition_id+"} "
      if len(self.dependency_id) > 0 :
        gaitstr+= "("+self.dependency_id+") "
      gaitstr+=";"
      return gaitstr
  ## Add extra information to this gait
  # @param self Object pointer
  # @param info_str String as the extra information
  def AddExtraInfo(self,info_str):
    self.ExtraInfo = info_str
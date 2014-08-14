#------------------------------------------------------------------------------#
#                                                                              #
#               Run-time error handler for Rapyd projects                      #
#                                                                              #
#------------------------------------------------------------------------------#

# This module handles run-time errors in order to report error locations in 
# terms of project module, form and line number. When your project is being run 
# from within Rapid this module facilitates the process whereby you can jump 
# directly to any project line shown in the error traceback.
#
# The file <projectname>.dec, which was generated when your project was built, is 
# required by this module in order to report error locations properly.

from Tkinter import *
import os
import os.path
import sys
import time
import traceback

def D(*Stuff):
    """=u
    This simply prints the arguments. 
    
    The point of using this, rather than 'print', is that once were done debugging 
    we can easily search for and remove the 'D' statements.
    """
    for T in Stuff:
        print T,
    print

def Grabber(Widget,Whine=0):
    """=u
    A persistent grabber
    
    For unknown reasons it sometimes takes awhile before the grab is successful;
        perhaps you have to wait until the window has finished appearing. In
        any case this persistent grabber loops for up to 0.3 seconds waiting for
        the grab to take before giving up.
    """
    for J in range(3):
        time.sleep(0.1)
        try:
            Widget.grab_set()
            return
        except TclError:
            pass
    if Whine:
        print 'Grab failed'        

def PythonModuleFind(Name):
    """
    Walk the Python search path looking for a module.
    
    Given a simple file name, eg "Tkinter.py", we look for the first occurrance in the Python module 
        search path and return as the result the full path to the file including the file itself,
        eg "/usr/lib/python2.3/lib-tk/Tkinter.py". 
        
    If we don't find a matching file we return None.
    """
    for Path in sys.path:
        if Path:
            for FileName in os.listdir(Path):
                if FileName == Name:
                    Result = Path+'/'+Name
                    if os.path.isfile(Result):
                        return Result
    return None 

def ExtractName(Path):
    """
    Extract just the bare filename from a path.
    
    eg given "/spam/parrot/stringettes.py" we return "stringettes"
    """
    return os.path.splitext(os.path.split(Path)[1])[0]

class CallWrapper:
    """
    Call wrapper so we can display errors which occur in a callback in a
        Rapyd friendly manner.
    """
    def __init__(self, func, subst, widget):
	self.func = func
	self.subst = subst
	self.widget = widget

    # Calling back from Tk into python.
    def __call__(self, *args):
	try:
	    if self.subst:
		args = apply(self.subst, args)
	    return apply(self.func, args)
	except SystemExit, msg:
	    raise SystemExit, msg
	except:
	    #_reporterror(self.func, args)
            RunError(func=self.func,args=args)

EventNames = {
    2 : 'KeyPress',         15 : 'VisibilityNotify',   28 : 'PropertyNotify',
    3 : 'KeyRelease',       16 : 'CreateNotify',       29 : 'SelectionClear',
    4 : 'ButtonPress',      17 : 'DestroyNotify',      30 : 'SelectionRequest',
    5 : 'ButtonRelease',    18 : 'UnmapNotify',        31 : 'SelectionNotify',
    6 : 'MotionNotify',     19 : 'MapNotify',          32 : 'ColormapNotify',
    7 : 'EnterNotify',      20 : 'MapRequest',         33 : 'ClientMessage',
    8 : 'LeaveNotify',      21 : 'ReparentNotify',     34 : 'MappingNotify',
    9 : 'FocusIn',          22 : 'ConfigureNotify',    35 : 'VirtualEvents',
    10 : 'FocusOut',        23 : 'ConfigureRequest',   36 : 'ActivateNotify',
    11 : 'KeymapNotify',    24 : 'GravityNotify',      37 : 'DeactivateNotify',
    12 : 'Expose',          25 : 'ResizeRequest',      38 : 'MouseWheelEvent',
    13 : 'GraphicsExpose',  26 : 'CirculateNotify',
    14 : 'NoExpose',        27 : 'CirculateRequest',
}

class RunError(Toplevel):
    """
    Dialog to handle error when user runs their project
    
    This dialog can be invoked in four different ways:
    
        o By direct call from inside Rapyd if a syntax error was encountered when 
              attempting to compile a user project prior to running it.
          This is mode: 0
          How we know: Rapyd passes us a locator dict as keyword argument
              "Info".      
          
        o From the users project which has been spawned as it's own process
              from Rapyd. 
          This is mode: 1.
          Now we know: Environment contains a variable named FROMRAPYD whose
              value is '*'.
        
        o From the users project which has been run standalone. 
          This is mode: 2.
          How we know: No "Info" argument and no FROMRAPYD environment variable.
        
        o From Rapyd, using execfile. Rapyd does this in order to check for
              syntax errors which would have prevented the project from running.
              If there are syntax-class errors then control passes back to rapyd
              which then invokes as described above for mode 0. However if, while
              being invoked via execfile, the main program itself is syntax error
              free but a run-time error is encountered or a syntax error happens
              in an imported file then we get control. As part of doing the execfile,
              Rapyd puts LocatorDict in our namespace, so it's just *there* without
              our having to do anything special
          This is mode: 3.
          How we know: Environment contains a variable named FROMRAPYD whose
              value is the path to the decoder file.      
        
        At startup we dig around and set self.mode per the above values.

    We need to get the locator dictionary so we can translate from python line numbers
        to Rapyd project line numbers. In Mode-0 Rapyd passes us the argument "Info"
        which contains the dictionary. In other modes we go looking for the file ----.dec
        (where ---- is the name of the project) and we build the locator dictionary from
        that file which was itself written when Rapyd built the project.
    
    In locator dictionary the key is the name of each generated module of the
        current project and the data is a locator list where each element is a 3-tuple:
        
        o [0] The name of the form or '-Main-'
        o [1] The offset in lines to where the first line from this form appears
              in the generated file.
        o [2] The number of lines from this form which appear in the generated form
              at this spot.
              
    If called directly from within rapyd (mode 0) then our result is returned in self.Result:
    
        o If user clicks "Dismiss" we return None
        o If user clicks "Pass error to Tkinter" we return 1
        o If user clicks on a line to go to we return a 3-list:
            [0] The name of the module of the errer
            [1] The name of the form of the error or '-Main-'
            [2] The line number of the error          
            
        When run from inside Rapyd we have access to the Rapyd help module and a user
        request for help we handle locally.
        
    If run as a distinct process by rapyd (mode 1) or if run while Rapyd was doing an
        "execfile" on the project (mode 3) then we return a result by writing to a
        file named "----.result" where ---- is the name of the project.       
        
        o If the user clicks on "Dismiss" we exit without creating any file.
        o If the user clicks on a line to go to then ----.result consists of 
          three elements:
          - The name of the module
          - The name of the form, or -Main-
          - The line number
        if The user clicks on Help the ----.results consists of "!HELP!".  
        
    If run as a stand-alone process (mode 2) then there is nobody to return a result to, 
        there is no help and error lines are not displayed as links because the user
        has no way to go directly to the code editor.
    """
    def __init__(self,Master=None,**kw):
        #
        # Get and save error information before any other error can occur
        #
        self.ErrorDescription = str(sys.exc_info()[1])

        #Each element of the traceback list is a tuple:
        #    o [0] Path and filename. Path may be an absolute path or may be relative
        #              to the current working directory.
        #    o [1] Line number
        #    o [2] Function
        #    o [3] Text
        self.TracebackList = traceback.extract_tb(sys.exc_info()[2])                    
        self.TypeOfError = str(sys.exc_info()[0])
        
        D('ErrorDescription=%s'%self.ErrorDescription)
        D('TracebackList=%s'%self.TracebackList)
        D('TypeOfError=%s'%self.TypeOfError)
        
        if '.' in self.TypeOfError:
            self.TypeOfError = self.TypeOfError.split('.')[1]

        if self.TypeOfError[-2:] == "'>":
            #Whereas previously Python would report the type of error as something like
            #    "NameError" as of around 2.5 it reports it as "<type 'exceptions.NameError'>"
            #    hence this code to shoot off the trailing crud.
            self.TypeOfError = self.TypeOfError[:-2]
        D('TypeOfError=%s'%self.TypeOfError)

        #
        # Any strings placed in ErrorNotes are displayed to the user prior to the error
        #     traceback.
        #
        ErrorNotes = []
        
        #
        # Look for callback related information
        #
        if kw.has_key('func'):
            #Looks like we were invoked from a callback
            self.func = kw['func']
            self.args = kw['args']
            del kw['func']
            del kw['args']
        else:
            self.func = None
            self.args = None    
        #
        # Get the locator dictionary and figure out what mode we are.
        #
        #D('Environ=%s'%os.environ)
        if kw.has_key('Info'):
            #We were called directly by Rapyd
            self.LocatorDict = kw['Info']
            del kw['Info']
            self.ProjectDirectory = kw['ProjectDirectory']
            del kw['ProjectDirectory']
            self.ResultPath = None
            self.Mode = 0
        elif os.environ.has_key('FROMRAPYD') and os.environ['FROMRAPYD'] <> '*':
            #We were invoked from within the project while Rapyd was doing an execfile
            #    on the project. In this case FROMRAPYD is the full path to the decoder
            #    file.
            FileName = os.environ['FROMRAPYD']
            self.ProjectDirectory = os.path.dirname(FileName) + '/'
            self.ResultPath = os.path.splitext(FileName)[0] + '.result'
            self.Mode = 3
        else:
            #We are running as a program on our own.
            self.ProjectDirectory = os.path.dirname(sys.argv[0])
            if self.ProjectDirectory <> '':
                self.ProjectDirectory += '/'
            FileName = os.path.splitext(sys.argv[0])[0]+'.dec'
            if os.environ.has_key('FROMRAPYD'):
                #But that program was invoked by Rapyd
                self.ResultPath = os.path.splitext(FileName)[0] + '.result'
                self.Mode = 1
            else:
                #We are totally stand-alone
                self.ResultPath = None
                self.Mode = 2    
        if self.Mode <> 0:
            #In mode 0 we are handed LocatorDict on a silver platter. For all other modes we
            #    have to go fetch the decoder file and turn it into LocatorDict. How we get
            #    the path to the decoder file varies by mode, but at this point it should be
            #    in "FileName".
            self.LocatorDict = {}
            #D('About to look for decoder file at: %s'%FileName)
            if os.path.isfile(FileName):
                try:
                    F = open(FileName)
                    Temp = F.readlines()
                    F.close()
                    for Line in Temp:
                        Line = Line.rstrip()
                        if len(Line) == 0:
                            raise Exception, 'Empty line'
                        if Line[0] <> ' ':
                            ModuleName = Line
                            self.LocatorDict[ModuleName] = []
                        else:
                            Line = Line.strip().split()
                            self.LocatorDict[ModuleName].append((Line[0], int(Line[1]), int(Line[2])))
                except:
                    self.LocatorDict = 'Error reading "%s"'%FileName
                    ErrorNotes.append('Unable to display module/form information due to error reading "%s"'%FileName)
            else:        
                ErrorNotes.append('Unable to display module/form information; file "%s" not found.'%FileName)
                self.LocatorDict = 'Not found'
        ##D('Mode=%s, LocatorDict=%s, ProjectDirectory=%s'%(self.Mode,self.LocatorDict,self.ProjectDirectory))

        apply(Toplevel.__init__,(self,Master),kw)
        self.title('%s in project'%self.TypeOfError)

        #Place the dialog in the center of the screen.
        Width, Height = (750,400)
        ScreenWidth = self.winfo_screenwidth()
        ScreenHeight = self.winfo_screenheight()
        X = (ScreenWidth-Width)/2
        Y = (ScreenHeight-Height)/2
        self.geometry('%sx%s+%s+%s'%(Width,Height,X,Y))

        self.Result = None
        #
        # Text widget for the traceback
        #
        self.T = Text(self,height=25)
        self.T.pack(expand=YES,fill=BOTH)

        self.T.tag_configure('Link', foreground='#009000')
        self.T.tag_bind('Link','<ButtonRelease-1>',self.on_HyperLink)

        if self.TypeOfError in ('IndentationError','SyntaxError'):
            #Syntax-class errors are the poor cousin of Python errors. For all other types of error
            #    we get an entry in the traceback list but for syntax errors all we get is
            #    the error description the form "invalid syntax (filename, line n)". Here we
            #    extract the filename and line number from the error description so we can add a
            #    standard-form entry to the tracback list.
            
            #In the code that follows:
            #    Line is the line number of the offending line
            #    Path is the path to the file that contains the offending line
            #    TheLine is the text of the offending line
            
            #Extract filename and line (as in integer)
            Filename,Line = self.ExtractStuff(self.ErrorDescription)
            #Filename is just that, a filename with no path. It may be part of our project or it may 
            #    be a file that our project includes. The locator dict keys mention all modules in 
            #   our project so we scan them to see if the file is part of our project.
            if type(self.LocatorDict) <> type({}):
                #We were not able to read the locator dictionary
                TheLine = '??????'
                Path = Filename
            elif Filename[:1] == '?':
                #Some versions of python (eg 2.2 under windows) do not tell us the filename.
                Path = Filename
                TheLine = "<<Python did not report the name of the file in which the error was found.>>"
            else:
                #We have the locator dictionary    
                for P in self.LocatorDict.keys():
                    if Filename == P:
                        #We found it; it's one of us.
                        Path = self.ProjectDirectory + Filename
                        break
                else:
                    #We didn't find it; walk the python module path looking for it.
                    Path = PythonModuleFind(Filename)
                    #Note that if we didn't find it in the PythonModule path either then
                    #    Path will be None at this point.
                if Path:
                    #We think we have a valid path
                    try:
                        F = open(Path,'r')
                        FileText = F.readlines()
                        F.close()
                    except:
                        FileText = ['[Unable to display line. An error happened while attempting to open "%s"]'%Path]
                    if len(FileText) < Line:
                        #The line is off the end; give them the last line
                        TheLine = FileText[-1].strip()
                    elif Line < 1:
                        print 'rpErrorHandler: Line is unexpectedly %s'%Line
                        TheLine = '<unavailable>'
                    else:        
                        TheLine = FileText[Line-1].strip()
                else:
                    #No valid path
                    Path = Filename
                    TheLine = '?????'    
            self.TracebackList.append((Path,Line,'?',TheLine))
            self.ErrorDescription = '' #No need to show line number twice
            
        #If there were any error notes, display them first
        for Note in ErrorNotes:
            self.T.insert(INSERT,'Note: %s\n'%Note)
        #
        # Report possible callback information
        #
        if self.func:
            self.T.insert(INSERT,'Exception in Tk callback\n')
            self.T.insert(INSERT,'  Function: %s (type: %s)\n'%(repr(self.func), type(self.func)))
            self.T.insert(INSERT,'  Args: %s\n'% str(self.args))
            
        #Figure out if the argument was an event
        EventArg = type(self.args)==type(()) and len(self.args) > 0 and hasattr(self.args[0],'type')    

        #Display the traceback list
        LinkCount = 0
        self.T.insert(INSERT,'Traceback (most recent call last):\n')

        #For some modes the first traceback entry is the call from Rapyd which not really
        #    of much use so we delete it.
        if self.Mode in (0,1,2):
            self.TracebackList = self.TracebackList[1:]
        
        for File,Line,Func,LineText in self.TracebackList:
            Module, Form,FormLine = self.Convert(File,Line)
            self.T.insert(INSERT,'    Module %s, Form %s, Line %s in %s  (File %s, line %s)\n'
                %(Module,Form,FormLine,Func,File,Line))
            if self.Mode in (0,1,3) and not ((Module[0] in '?<') or (File[0]=='?')):
                #We have been invoked from Rapyd and the subject line is in our project.
                #Set tags so the text will be a clickable link.
                Tags = 'Link =%s:%s:%s'%(Module,Form,FormLine)
                self.T.config(cursor='hand2')
                LinkCount += 1
            else:
                #This text is not jumptoable
                Tags = None    
            self.T.insert(INSERT,'        %s\n'%LineText,Tags)        
        self.T.insert(INSERT,'%s: %s\n'%(self.TypeOfError,self.ErrorDescription))
        #    
        #If we were able to display lines as links, give the user a heads up.
        #
        if LinkCount > 0:
            if LinkCount == 1:
                Msg = "\n(click on the green line above to go to that line in the corresponding code editor)\n"
            else:
                Msg = "\n(click on a green line above to go to that line in the corresponding code editor)\n"
            self.T.insert(INSERT,Msg)    
        #
        # If we have an event display some information about it
        #
        self.T.insert(INSERT,'\n')
        if EventArg:
            EventNum = int(self.args[0].type)
            if EventNum in EventNames.keys():
                self.T.insert(INSERT,'Event type: %s (type num: %s).  Event content:\n'
                    %(EventNames[EventNum], EventNum))
            Keys = self.args[0].__dict__.keys()
            Keys.sort()
            for Key in Keys:
                self.T.insert(INSERT,'  %s: %s\n'%(Key, self.args[0].__dict__[Key]))        

        #
        # Button bar
        #        
        self.Buttons = Frame(self)
        self.Buttons.pack(side=BOTTOM)
        Button(self.Buttons,text='Dismiss',command=self.on_OK).pack(side=LEFT,padx=10,pady=5)
        if self.Mode in (0,1,3):
            Button(self.Buttons,text='Help',command=self.on_Help).pack(side=LEFT,padx=10,pady=5)
        self.bind('<Return>',self.on_OK)
        self.bind('<Escape>',self.on_OK)
        self.bind('<F1>',self.on_Help)
        #
        #be modal
        #
        self.focus_set()
        Grabber(self)
        self.wait_window()
        
    def on_OK(self,Event=None):
        """
        User clicked on Dismiss
        """
        self.Result = None
        if self.Mode == 3:
            #We have to pass the result back via a file
            try:
                F = open(self.ResultPath,'w')
                F.write('!HANDLED!')
                F.close()
            except:
                print 'rpErrorHandler: Error writing result file'    
        self.destroy()
        
    def on_Help(self,Event=None):
        """
        User asked for help.
        """
        if self.Mode == 0:
            Help('run-error-dialog')
        elif self.Mode in (1,3):
            #We have to pass the result back via a file
            try:
                F = open(self.ResultPath,'w')
                F.write('!HELP!')
                F.close()
            except:
                print 'rpErrorHandler: Error writing result file'    
            if self.Mode == 1:
                #If we are an actual process spawned by Rapyd then wrap up our process.
                self.quit()
            self.destroy()

    def on_HyperLink(self,Event=None):
        """
        User clicked on an error line which was rendered as a clickable link
        """
        #Get tags associated with the cursor position
        Tags = Event.widget.tag_names(CURRENT)
        for T in Tags:
            #look for a tag that starts with equal-sign. The rest of the tag will
            #    be "module:form:line"
            if T[0:1] == '=':
                self.Result = T[1:].split(':')
                self.Result[2] = int(self.Result[2])
                if self.Mode in (1,3):
                    #We have to pass the result back via a file
                    try:
                        F = open(self.ResultPath,'w')
                        F.write('%s %s %s'%tuple(self.Result))
                        F.close()
                    except:
                        print 'rpErrorHandler: Error writing result file'    
                    if self.Mode == 1:    
                        self.quit()    
                self.destroy()        

    def ExtractStuff(self,Line):
        """
        Extract some stuff from a line
        
        Given a line of the form:
        
            "syntax error (ffff, line nnnn)"
            
        Return ['ffff',nnnn]
        
        Note: Some versions of Python (eg python2.2 under windows) return a line of the form
            "syntax error (line nnnn)" without giving us the file. Without the file it's a
            tad hard to fetch the offending line, but that's the way it is. In that case,
            we return "????" in lieu of the filename as ffff.
        """
        T = Line.split('(')[1] #ffff, line nnnn)
        print 'T=%s'%T
        T = T.split(',') #['ffff',' line nnnn)'] 
        print 'T=%s'%T
        if len(T) == 2:
            #a file was specified
            ffff = T[0]
            T = T[1].strip() #'line nnnn)'
        else:
            #no file was specified
            T = T[0]    
            ffff = "????"
        T = T.split(' ')[1] # 'nnnn)'
        nnnn = T[:-1]
        return [ffff,int(nnnn)]


    def Convert(self,Filename,LineNumber):
        """
        Convert a generated file reference to a rapyd reference.
        
        Filename is a path to the generated python file.
        Linenumber is the line number in the generated file as returned by
            python in origin-1
        
        The result is a 3-list giving:
            o [0] The name of the module of the rapid project.
            o [0] The form (or '-Main-') in the module project.
            o [1] The line number in the said form in origin-1.
                
        If the file is not part of our project we return ('<None>','<None>',0)        
        """
        if type(self.LocatorDict) <> type({}):
            #We don't have a valid locator dict
            return ['?????','?????',0]
            
        Filename = os.path.split(Filename)[1]
        ##D('Convert: Filename=%s, LineNumber=%s'%(Filename,LineNumber))
            
        try:
            Locator = self.LocatorDict[Filename]
        except:
            return ['<None>','<None>',0]    
        MainChunkOffset = 0
        ModuleName = os.path.splitext(Filename)[0]
        ##D('=== seeking %s in %s'%(LineNumber,Filename))
        
        for Form,Offset,Length in Locator:
            ##D('Form=%s Offset=%s Length=%s MainChunkOffset=%s'%(Form,Offset,Length,MainChunkOffset))
            if Offset+Length > LineNumber-1:
                Result = [ModuleName, Form, LineNumber-Offset]
                if Form == '-Main-':
                    Result[2] += MainChunkOffset
                return Result    
            if Form == '-Main-' and MainChunkOffset == 0:
                #This little dance with MainChunkOffset is necessary because the 
                #    -Main- code is split up into two chunks with the form
                #    code in between them.
                MainChunkOffset = Length    
        #They asked for a line off the end. Give them the last line.
        return [ModuleName, Form, Length+Offset]
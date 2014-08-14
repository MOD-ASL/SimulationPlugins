#!/usr/bin/python

import rpErrorHandler
from Tkinter import *
#------------------------------------------------------------------------------#
#                                                                              #
#                                  ControlGUI                                  #
#                                                                              #
#------------------------------------------------------------------------------#
class ControlGUI(Frame):
    def __init__(self,Master=None,**kw):
        #
        #Your code here
        #

        apply(Frame.__init__,(self,Master),kw)
        self.__Frame9 = Frame(self,width=30)
        self.__Frame9.pack(side='left')
        self.__Frame10 = Frame(self)
        self.__Frame10.pack(side='left')
        self.__Frame3 = Frame(self)
        self.__Frame3.pack(side='left')
        self.__Frame5 = Frame(self.__Frame10)
        self.__Frame5.pack(side='top')
        self.__Listbox_configuration = Listbox(self.__Frame5,height=20,width=35)
        self.__Listbox_configuration.pack(fill='both',side='top')
        self.__Frame1 = Frame(self.__Frame10)
        self.__Frame1.pack(side='top')
        self.__Button_importConfig = Button(self.__Frame1
            ,text='Import Configuration')
        self.__Button_importConfig.pack(side='top')
        self.__Frame4 = Frame(self.__Frame3)
        self.__Frame4.pack(side='top')
        self.__Frame2 = Frame(self.__Frame3)
        self.__Frame2.pack(side='top')
        self.__Button_executeGait = Button(self.__Frame2,text='Execute Gait')
        self.__Button_executeGait.pack(side='left')
        self.__Button_stopGait = Button(self.__Frame2,text='Stop Gait')
        self.__Button_stopGait.pack(side='right')
        self.__Frame7 = Frame(self.__Frame4,width=10)
        self.__Frame7.pack(side='left')
        self.__Frame8 = Frame(self.__Frame4)
        self.__Frame8.pack(side='left')
        self.__Listbox_gait = Listbox(self.__Frame8,height=20,width=35)
        self.__Listbox_gait.pack(fill='both',side='top')
        #
        #Your code here
        #
    #
    #Start of event handler methods
    #

    #
    #Start of non-Rapyd user code
    #


try:
    #--------------------------------------------------------------------------#
    # User code should go after this comment so it is inside the "try".        #
    #     This allows rpErrorHandler to gain control on an error so it         #
    #     can properly display a Rapyd-aware error message.                    #
    #--------------------------------------------------------------------------#

    #Adjust sys.path so we can find other modules of this project
    import sys
    if '.' not in sys.path:
        sys.path.append('.')
    #Put lines to import other modules of this project here
    
    if __name__ == '__main__':

        Root = Tk()
        import Tkinter
        Tkinter.CallWrapper = rpErrorHandler.CallWrapper
        del Tkinter
        App = ControlGUI(Root)
        App.pack(expand='yes',fill='both')

        Root.geometry('640x480+10+10')
        Root.title('ControlGUI')
        Root.mainloop()
    #--------------------------------------------------------------------------#
    # User code should go above this comment.                                  #
    #--------------------------------------------------------------------------#
except:
    rpErrorHandler.RunError()
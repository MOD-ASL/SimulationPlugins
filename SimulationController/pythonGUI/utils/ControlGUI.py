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
        self.__Frame6 = Frame(self.__Frame10)
        self.__Frame6.pack(side='top')
        self.__Button_loadWorld = Button(self.__Frame6,text='Load World')
        self.__Button_loadWorld.pack(side='left')
        self.__Button_loadLibrary = Button(self.__Frame6,text='Load Library')
        self.__Button_loadLibrary.pack(side='right')
        self.__Frame4 = Frame(self.__Frame3)
        self.__Frame4.pack(side='top')
        self.__Frame11 = Frame(self.__Frame3)
        self.__Frame11.pack(side='top')
        self.__Frame2 = Frame(self.__Frame3)
        self.__Frame2.pack(side='top')
        self.__Frame7 = Frame(self.__Frame4,width=10)
        self.__Frame7.pack(side='left')
        self.__Label_scoreText = Label(self.__Frame7,text='Score')
        self.__Label_scoreText.pack(side='top')
        self.__Label_scoreNum = Label(self.__Frame7,width=20)
        self.__Label_scoreNum.pack(side='top')
        self.__Frame8 = Frame(self.__Frame4)
        self.__Frame8.pack(side='left')
        self.__Listbox_gait = Listbox(self.__Frame8,height=20,width=35)
        self.__Listbox_gait.pack(fill='both',side='top')
        self.__Frame12 = Frame(self.__Frame11)
        self.__Frame12.pack(ipadx=20,padx=60,side='left')
        self.__Frame13 = Frame(self.__Frame11)
        self.__Frame13.pack(side='left')
        self.__Button_executeGait = Button(self.__Frame13,text='Execute Gait')
        self.__Button_executeGait.pack(side='top')
        self.__Frame15 = Frame(self.__Frame2)
        self.__Frame15.pack(ipadx=20,padx=60,side='left')
        self.__Frame14 = Frame(self.__Frame2)
        self.__Frame14.pack(side='left')
        self.__Button_close = Button(self.__Frame14,text='Close')
        self.__Button_close.pack(side='top')
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
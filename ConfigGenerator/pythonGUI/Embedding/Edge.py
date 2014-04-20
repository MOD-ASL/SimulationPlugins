'''
Created on Dec 28, 2013

@author: tariktosun
'''

import Node

class Edge(object):
    '''
    Edge class for design embedding.
    '''


    def __init__(self, parent, child, joint):
        '''
        parent and child are Nodes.
        '''
        # def __init__(self, parent, child, length):
        assert isinstance(parent, Node.Node)
        assert isinstance(child, Node.Node)
        
        # The parent node
        self.parent = parent
        # The child node
        self.child = child
        # Edge length
        #self.length = length
        
        ## Kinematic stuff:
        #assert frame.p.Norm() == length, 'Length and frame vector norm are not the same.'
        self.joint = joint
        self.current_angle = 0
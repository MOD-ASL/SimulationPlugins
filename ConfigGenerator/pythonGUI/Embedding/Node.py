'''
Created on Dec 28, 2013

@author: tariktosun
'''
#from Embedding.Edge import Edge
import roslib
roslib.load_manifest('kdl')
from PyKDL import *
from Edge import Edge

class Node(object):
    '''
    Node class for design merging
    '''

    def __init__(self, name):
        '''
        Constructor
        '''
        # Name:
        self.name = name
        # List of pointers to child nodes
        self.children = []
        # Edge linking this node to its parent
        self.parent_edge = None
        # Pointer to parent node
        self.parent = None
        # Type abstractly captures functionality (for now)
        self.type = None
        # Nodes that are end effectors  cannot have children.
        self.is_end_effector = False
        # Nodes for which active == True are considered in embeddings; inactive
        # nodes are stripped.
        self.active = True  # note: inactive nodes must be set as such manually.
        
        #         ## Kinematic stuff:
        #         self.joint = node_joint # should be a KDL Joint object.
        #         self.current_angle = 0
        #         self.child_edges = []
        
        ## New kinematic stuff:
        self.child_frames = {} # Frames to child edges.
    
    def nodecost(self):
        '''
        Returns the cost of this node.
        '''
        pass
    
    def add_child(self, child, frame, joint):
        '''
        Builds an edge of specified length from child to this node, making it a
        child of this node.
        '''
        # def add_child(self, child, length=0):
        assert type(frame) == Frame
        assert type(joint) == Joint
        self.children.append(child)
        self.child_frames[child] = frame
        e = Edge(self, child, joint)
        child.parent = self
        child.parent_edge = e
        return
        
    def remove_child(self, child):
        '''
        Breaks edge between this node and child.
        '''        
        if child in self.children:
            self.children.remove(child)
            del self.child_frames[child]
        else:
            raise Exception('Child removal failed.  Node was not in list of children.')
        if child.parent == self:
            child.parent = None
            child.parent_edge = None
            # Note: edge is now orphaned.
        else:
            raise Exception('Child removal failed.  Node was not child\'s parent.')
    
    def __repr__(self):
        params = {}
        params['name'] = self.name
        params['children'] = str([c.name for c in self.children])
        if self.parent:
            params['parent_name'] = str(self.parent.name)
        else:
            params['parent_name'] = 'None'
        params['type'] = str(self.type)
        return 'Node %s:\n children: %s\n parent: %s\n type: %s' % (params['name'],
                                                                params['children'],
                                                                params['parent_name'],
                                                                params['type'],
                                                                )
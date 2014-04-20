'''
Created on Jan 15, 2014

@author: tariktosun
'''
import roslib
roslib.load_manifest('kdl')
from PyKDL import * 
from math import pi
import Node

class SmoresModule(object):
    '''
    Object representing a SMORES robot module for design synthesis project.
    A SMORES module is composed of four nodes, which are numbered as follows:
            0
            |
        2 - 1 - 3
            |
            4
    Edge 0-1, 1-2, and 1-3 are CR, and edge 1-4 is a hinge.  All nodes are type
    2. Node 1 is the only one with extent. All nodes are active by default.  The
    user may specify which nodes are inactive using arguments to the constructor
    and add_child functions.   
    '''

    def __init__(self, name, root_node_number, inactive_nodes_numbers=[]):
        '''
        Constructor.  root_node is a number (0-4) specifying which node in this
        module is the root.  inactive_nodes is a list of numbers (again, 0-3)
        specifying which nodes should be considered inactive in this module.
        '''
        # name:
        self.name = name
        # root node number:
        self.root_node_number = root_node_number
        # list of child modules:
        self.child_modules = [None, None, None, None, None]    # note that only 3 may be used.
        # parent module:
        self.parent_module = None
        
        # set up list of nodes.  The order is important.
        self.nodes = [Node.Node(self.name+"-"+str(i)) for i in range(5)]
        # set activity:
        for i in inactive_nodes_numbers:
            self.nodes[i].active = False
        # set types:
        for i in range(0,5):
            self.nodes[i].type = 2
        # create structure based on root:
        self.base_frame = self._get_root2base(root_node_number)
        n = self.nodes # for easy typing...
        bfInR = self.base_frame
        cInB = self._get_base2child # This is a function.
        J = self._get_node_parent_joint #this is a function.
        if root_node_number == 0:
            n[0].add_child( n[1], Frame(), Joint(Joint.RotX))
            for i in [2,3,4]:
                n[1].add_child( n[i], bfInR*cInB(i), Joint(J(i)) )
        if root_node_number == 1:
            assert False, 'Node 1 cannot be the root.'
        if root_node_number == 2:
            n[2].add_child( n[1], Frame(), Joint(Joint.RotX) )
            for i in [0, 3, 4]:
                n[1].add_child( n[i], bfInR*cInB(i), Joint(J(i)) )
        if root_node_number == 3:
            n[3].add_child( n[1], Frame(), Joint(Joint.RotX) )
            for i in [0,2,4]:
                n[1].add_child( n[i], bfInR*cInB(i), Joint(J(i)) )
        if root_node_number == 4:
            n[4].add_child( n[1], Frame( Vector(1, 0, 0)), Joint(Joint.RotY) )
            for i in [0, 2, 3]:
                #n[1].add_child( n[i], bfInR*cInB(i), Joint(J(i)))
                n[1].add_child( n[i], cInB(i), Joint(J(i)))
            
    def add_child_module(self, node_number, child_module):
        '''
        Adds a child module to this module, connected at specified node number.
        '''
        assert isinstance(child_module, SmoresModule), 'child_module is not a SmoresModule object.'
        if self.parent_module is not None:
            assert node_number is not self.root_node_number, 'Attempted to add child module at root_node_number to non-root module.'
            assert node_number is not 1, 'Cannot add child_module at node 1.'
        assert self.child_modules[node_number] is None, 'Attempted to add a child to a filled node_number.'
        # Add at the module level:
        self.child_modules[node_number] = child_module
        child_module.parent_module = self
        # connect root node of child module to specified node of this module:
        # Rather than adding a rigid connection, we're going to fuse two nodes.
        n = self.nodes[node_number] #node in the parent which will be fused.
        p = n.parent
        r = child_module.nodes[child_module.root_node_number] # root node in the child module (to be fused).
        # fuse names:
        r.name = n.name + '/' + r.name
        # connect n's parent node to r:
        p.children.append(r)
        p.children.remove(n)
        p.child_frames[r] = p.child_frames[n]
        del p.child_frames[n]
        r.parent = p
        # handle the edge:
        n.parent_edge.child = r
        r.parent_edge = n.parent_edge
        # handle node list in the module:
        self.nodes[node_number] = r # node n is no longer needed; it has been fused into r.
        # Add a frame xform if needed:
        if node_number == 4:
            assert len(r.child_frames.keys())==1, 'Fusion error: child module root node has more than one child!'
            childInR = r.child_frames.values()[0]
            rInN = Frame(Vector(1,0,0))
            r.child_frames[r.child_frames.keys()[0]] = rInN*childInR

        #self.nodes[node_number].add_child(child_module.nodes[child_module.root_node_number], length)
        
        
    def remove_child_module(self, child_module):
        '''
        Removes a child module from this module.
        '''
        assert child_module in self.child_modules, 'child_module was not in this module\'s list of children.'
        assert child_module.parent_module is self, 'This module was not child_module\'s  parent.'
        # remove from list of child modules:
        child_module_number = self.child_modules.index(child_module)
        self.child_modules[child_module_number] = None
        # Break the connection between nodes:
        self.nodes[child_module_number].remove_child(child_module.parent_module)
    
    def _get_root2base(self, root_node_number):
        '''
        Returns the Frame transform from the root node to the base frame.
        '''
        if root_node_number is 0:
            return Frame( Rotation.RPY(0, 0, pi), Vector(1, 0, 0) )
        elif root_node_number is 1:
            assert False, 'node 1 may not be root.'
        elif root_node_number is 2:
            return Frame( Rotation.RPY(0, 0, pi/2), Vector(1, 0, 0) )
        elif root_node_number is 3:
            return Frame( Rotation.RPY(0, 0, pi/2), Vector(1, 0, 0) )
        elif root_node_number is 4:
            return Frame( Vector(1, 0, 0) )
        else:
            assert False, 'node number ' + str(root_node_number) + ' is invalid.'
            
    def _get_base2child(self, child_node_number):
        '''
        Returns Frame transform from base frame to a child node.
        '''
        if child_node_number is 0:
            return Frame( Vector(1, 0, 0) )
        elif child_node_number is 1:
            assert False, 'node 1 frame is determined by root.'
        elif child_node_number is 2:
            return Frame( Rotation.RPY(0, 0, pi/2), Vector(0, 1, 0) )
        elif child_node_number is 3:
            return Frame( Rotation.RPY(0, 0, -pi/2), Vector(0, -1, 0) )
        elif child_node_number is 4:
            return Frame( Rotation.RPY(0, 0, pi ) )
        else:
            assert False, 'node number ' + str(child_node_number) + ' is invalid.'
            
    def _get_node_parent_joint(self, child_node_number):
        '''
        Returns the appropriate parent joint type for this child node.
        '''
        if child_node_number is 0:
            return Joint.RotX
        elif child_node_number is 1:
            assert False, 'invalid for node 1'
        elif child_node_number is 2:
            return Joint.RotX
        elif child_node_number is 3:
            return Joint.RotX
        elif child_node_number is 4:
            return Joint.RotY
        else:
            assert False, 'node number ' + str(child_node_number) + ' is invalid.'
        
        
        
        
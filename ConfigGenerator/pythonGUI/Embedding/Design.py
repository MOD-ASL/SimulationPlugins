'''
Created on Dec 29, 2013

@author: tariktosun
'''
#from Embedding.Node import Node

import Node
import copy
import roslib
roslib.load_manifest('kdl')
from PyKDL import *
from math import pi

class Design(object):
    '''
    A design, composed of nodes and edges.
    '''

    def __init__(self, root_node, node_list):
        '''
        Constructor
        '''
        assert isinstance(root_node, Node.Node)
        
        # list of nodes in this design
        self.nodes = node_list
        # parse tree to extract edges and ensure design is valid.
        nodes = []
        edges = []
        self.parse_tree(root_node, nodes, edges)
        # ensure node_list and nodes have the same nodes:
        assert set(self.nodes) == set(nodes), 'Mismatch in tree and node_list'
        # for now, the design just gets its edges from the nodes.
        self.edges = edges
        self.root_node = root_node
        
    def parse_tree(self, root_node, nodes, edges):
        '''
        Recursive function which extracts all nodes and edges from a connected set of nodes.
        '''
        assert not root_node in nodes, "Design tree contains a cycle."
        nodes.append(root_node)
        if root_node.parent_edge:
            edges.append(root_node.parent_edge)
        for child in root_node.children:
            self.parse_tree(child, nodes, edges)
        return
    
    def strip_inactive_nodes(self):
        '''
        Returns a design identical to this one, but with inactive nodes stripped.
        Note that this does not modify the current Design, it returns a new one.
        '''
        #stripped_design = copy.deepcopy(self)
        inactive_nodes = [n for n in self.nodes if n.active == False]
        for n in inactive_nodes:
            assert n.is_end_effector == False, 'Attempted to strip an inactive node that was an end effector'
            assert n.children == [], 'Attempted to strip inactive node ' + str(n.name) + ', but it had children'
            self.edges.remove(n.parent_edge)
            self.nodes.remove(n)
            p = n.parent
            p.remove_child(n)
        #return stripped_design
    
    def get_kinematics(self, parent_node, child_node, universal_root=False):
        '''
        Returns (chain, jointAngles), where chain is a KDL chain corresponding 
        to the path between parent_node and child_node, and jointAngles is a
        KDL JntArray object with the current angles of the joints along the chain.
        '''
        # produce a stack of joints, a stack of frames, and a stack of angles.
        frame_stack = []
        joint_stack = []
        angles_stack = []
        # Append child with Identity frame:
        #joint_stack.append( Joint(Joint.None) )
        joint_stack.append( child_node.parent_edge.joint )
        #frame_stack.append( Frame( Rotation.Identity() )) wrong.    
        joint_type = child_node.parent_edge.joint.getType()
        if joint_type == Joint.RotX: #X
            frame_stack.append( Frame( Rotation.Identity() ) )
        elif joint_type == Joint.RotY: #Y
            frame_stack.append( Frame( Rotation.RPY(0, 0, pi/2) ) )
        elif joint_type == Joint.RotZ: #Z
            frame_stack.append( Frame( Rotation.RPY(0, -pi/2, 0) ) )
        angles_stack.append( child_node.parent_edge.current_angle )
        p = child_node
        while p is not parent_node:
            if p.parent is None:
                return False    # design root reached
            #frame_stack.append( p.parent_edge.frame )
            frame_stack.append( p.parent.child_frames[p] )
            p = p.parent
            if p.parent_edge is None: #if it's none, it's the root, and this is ok.
                assert universal_root
            else:
                joint_stack.append( p.parent_edge.joint )
                if not p.parent_edge.joint.getType() == Joint.None:
                    angles_stack.append(p.parent_edge.current_angle)
        # Now append initial rotation for parent:
        if universal_root:
            # Handles root node corner case.  Parent joint of chain is a u-joint.
            joint_stack.append(Joint(Joint.RotX))
            angles_stack.append(0)
            frame_stack.append(Frame())
            joint_stack.append(Joint(Joint.RotY))
            angles_stack.append(0)
            frame_stack.append(Frame())
            joint_stack.append(Joint(Joint.RotZ))
            angles_stack.append(0)
            if len(frame_stack) < len(joint_stack):
                # Add in another frame if we need one.
                frame_stack.append(Frame())
        else:
            joint_stack.append( Joint(Joint.None) )
            parent_joint_type = parent_node.parent_edge.joint.getType()
            if parent_joint_type == Joint.RotX: #X
                frame_stack.append( Frame( Rotation.Identity() ) )
            if parent_joint_type == Joint.RotY: #Y
                frame_stack.append( Frame( Rotation.RPY(0,0,-pi/2) ) )
            if parent_joint_type == Joint.RotZ: #Z
                frame_stack.append( Frame( Rotation.RPY(0,pi/2,0) ) )
        
        # Pop off the stack to populate chain and jointAngles:
        jointAngles = JntArray( len(angles_stack) )
        chain = Chain()
        i = 0
        while len(joint_stack) > 0:
            joint = joint_stack.pop()
            frame = frame_stack.pop()
            if not joint.getType() == Joint.None:
                angle = angles_stack.pop()
                jointAngles[i] = angle
                i += 1
            segment = Segment( joint, frame )
            chain.addSegment( segment )
        assert len(angles_stack) == 0
        assert len(frame_stack) == 0
        assert chain.getNrOfJoints() == jointAngles.rows()
        return (chain, jointAngles)
    
    
    #     def get_kinematics(self, parent_node, child_node):
    #         '''
    #         Returns (chain, jointAngles), where chain is a KDL chain corresponding 
    #         to the path between parent_node and child_node, and jointAngles is a
    #         KDL JntArray object with the current angles of the joints along the chain.
    #         '''
    #         # produce a stack of joints, a stack of frames, and a stack of angles.
    #         frame_stack = []
    #         joint_stack = []
    #         angles_stack = []
    #         # Append child with Identity frame:
    #         #joint_stack.append( Joint(Joint.None) )
    #         joint_stack.append( child_node.joint )
    #         #frame_stack.append( Frame( Rotation.Identity() )) wrong.    
    #         child_node_type = child_node.joint.getType()
    #         if child_node_type == Joint.RotX: #X
    #             frame_stack.append( Frame( Rotation.Identity() ) )
    #         elif child_node_type == Joint.RotY: #Y
    #             frame_stack.append( Frame( Rotation.RPY(0, 0, pi/2) ) )
    #         elif child_node_type == Joint.RotZ: #Z
    #             frame_stack.append( Frame( Rotation.RPY(0, -pi/2, 0) ) )
    #         angles_stack.append( child_node.current_angle )
    #         p = child_node
    #         while p is not parent_node:
    #             if p.parent is None:
    #                 return False    # design root reached
    #             frame_stack.append( p.parent_edge.frame )
    #             p = p.parent
    #             joint_stack.append( p.joint )
    #             if not p.joint.getType() == Joint.None:
    #                 angles_stack.append(p.current_angle)
    #         # Now append initial rotation for parent:
    #         joint_stack.append( Joint(Joint.None) )
    #         parent_node_type = parent_node.joint.getType()
    #         if parent_node_type == Joint.RotX: #X
    #             frame_stack.append( Frame( Rotation.Identity ) )
    #         if parent_node_type == Joint.RotY: #Y
    #             frame_stack.append( Frame( Rotation.RPY(0,0,-pi/2) ) )
    #         if parent_node_type == Joint.RotZ: #Z
    #             frame_stack.append( Frame( Rotation.RPY(0,pi/2,0) ) )
    #         
    #         # Pop off the stack to populate chain and jointAngles:
    #         jointAngles = JntArray( len(angles_stack) )
    #         chain = Chain()
    #         i = 0
    #         while len(joint_stack) > 0:
    #             joint = joint_stack.pop()
    #             frame = frame_stack.pop()
    #             if not joint.getType() == Joint.None:
    #                 angle = angles_stack.pop()
    #                 jointAngles[i] = angle
    #                 i += 1
    #             segment = Segment( joint, frame )
    #             chain.addSegment( segment )
    #         assert len(angles_stack) == 0
    #         assert len(frame_stack) == 0
    #         assert chain.getNrOfJoints() == jointAngles.rows()
    #         return (chain, jointAngles)
            
    def set_path_angles(self, parent_node, child_node, angles_array):
        '''
        Sets the current_angle of nodes along the path from parent to child to
        the angles specified in angles_array.
        '''
        # angle of child_node will be on top.
        angles_stack = [ angles_array[i] for i in xrange(angles_array.rows()) ]
        p = child_node
        #assert p.parent is not None, 'Design root reached'
        #p = p.parent    # we don't set the angle of the child node.
        while p is not parent_node:
            assert p.parent is not None, 'Design root reached'
            angle = angles_stack.pop()
            p.parent_edge.current_angle = angle
            p = p.parent
        # We do set the angle of the parent node:
        # (unless it is the root, in which case we don't do this.)
        if p.parent is not None:    
            angle = angles_stack.pop()
            p.current_angle = angle
        return
    
    def check_validity(self):
        '''
        Checks a number of conditions to ensure that this design is a valid design.
        Throws assertion error if design fails, runs silently otherwise.
        '''
        # ensure that design contains no cycles, and that list of nodes and edges
        # correspond to those actually connected to the root.
        nodes = []
        edges = []
        self.parse_tree(self.root_node, nodes, edges)
        assert set(self.nodes) == set(nodes), 'Mismatch in tree and node list'
        assert set(self.edges) == set(edges), 'Mismatch in tree and edge list'
        
        # ensure all edges have non-negative lengths, and that edge parents and
        # children correspond with the nodes they connect.
        for e in self.edges:
            #assert e.length >= 0, 'An edge has negative length'
            # edges no longer have length.
            assert e.child.parent == e.parent, 'Edge child parent is not edge parent.'
            assert e.child.parent_edge == e, 'Edge child\'s parent edge is not this edge.'
            
        # Check nodes:
        for n in self.nodes:
            if n.parent is not None:    # root has no parent.
                assert n in n.parent.children, 'node is not a child of its parent'
            for c in n.children:
                assert c.parent == n, 'node is not the parent of its child'
            if n.active == False:
                assert n.is_end_effector == False, 'An Inactive node is an end-effector'
                try:
                    assert n.children == [], 'An Inactive node has children.'
                except:
                    pass
            if n.is_end_effector:
                assert n.children == [], 'An End-effector has children.'
            
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        

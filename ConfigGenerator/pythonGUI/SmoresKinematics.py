from Embedding import SmoresModule, SmoresDesign
from PyKDL import *
import pdb

class SmoresKinematics(object):

	def __init__(self):
		self.module_list = None		# All modules in the cluster
		self.root_module = None		# Root module of the cluster
		self.root_node = None		# root node of the root module of the cluster
		self.name_mapping = None	# maps module names to internal module objects
		self.design = None			# SmoresDesign object corresponding to the cluster.
									# Re-generated every time a new module is added.

	def add_root_module(self, module_name, xyz_position, joint_angles):
		''' Adds the first module to the kinematic structure '''
		assert self.root_module == None
		assert self.module_list == None
		assert self.name_mapping == None
		self.name_mapping = {}
		root_module = SmoresModule.SmoresModule( module_name, 3 )	# root module always rooted at node 2
		self.root_module = root_module
		self.root_node = root_module.nodes[root_module.root_node_number]
		self.name_mapping[module_name] = root_module
		self.module_list = [root_module]
		self.set_joint_angles( root_module, joint_angles )
		# Create design and check validity
		self.design = SmoresDesign.SmoresDesign( root_module, self.module_list )
		self.design.check_validity();
		return

	def add_child_module(self, parent_module_name, new_module_name, parent_face, new_module_face, joint_angles):
		''' Adds a child module to the kinematic structure.  '''
		assert not self.name_mapping.has_key(new_module_name)
		assert self.name_mapping.has_key(parent_module_name)

		nodemap = [0,2,3,4]	# maps face numbering convention to node numbering convention
		parent_module = self.name_mapping[parent_module_name]
		assert not parent_module.nodes[nodemap[parent_face]] is self.root_node, "New modules may not be attached at the design root."

		new_module = SmoresModule.SmoresModule( new_module_name, nodemap[new_module_face] )
		self.name_mapping[new_module_name] = new_module
		self.module_list.append( new_module )
		parent_module.add_child_module( nodemap[parent_face], new_module )
		self.set_joint_angles( new_module, joint_angles )
		# re-create the design and check validity:
		self.design = SmoresDesign.SmoresDesign( self.root_module, self.module_list )
		self.design.check_validity
		return

	def get_module_position(self, module_name):
		''' returns the current position of the middle of the module of interest, relative the root. '''
		assert self.name_mapping.has_key(module_name)
		module = self.name_mapping[module_name]
		# Get the KDL chain of the path from the root to the module of interest.
		# Note that node 1 is at the center of the module.
		(kdl_chain, joint_angles) = self.design.get_kinematics( self.root_module.nodes[1], module.nodes[1] )
		fk = ChainFkSolverPos_recursive(kdl_chain)
		final_frame = Frame()
		fk.JntToCart(joint_angles, final_frame)
		# convert to world coordinates:


		# convert to a touple and return
		xyz = final_frame.p
		rpy = final_frame.M.GetRPY()
		return ( xyz.x(), xyz.y(), xyz.z(), rpy[0], rpy[1], rpy[2] )

	def set_joint_angles(self, module, joint_angles ):
		''' sets the joint angles of the module to those specified.
		We assume joints are passed in in order, and map joint0 to node 0,
		joint1 to node 2, joint2 to node 3, and joint3 to node 4.  See schematic in
		SmoresModule.py.'''
		nodemap = [0,2,3,4]
		for i in xrange(0,4):
			nodenumber = nodemap[i]
			if nodenumber == module.root_node_number:
				edge = module.nodes[1].parent_edge
			else:
				edge = module.nodes[nodemap[i]].parent_edge
			edge.current_angle = joint_angles[i]
		return




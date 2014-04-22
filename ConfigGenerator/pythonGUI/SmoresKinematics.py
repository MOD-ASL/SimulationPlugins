from Embedding import SmoresModule, SmoresDesign
from PyKDL import *

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
		self.name_mapping = {}
		root_module = SmoresModule.SmoresModule( module_name, 1 )	# rooted at node 1
		self.name_mapping[module_name] = root_module
		self.module_list = [root_module]
		self.set_joint_angles( root_module, joint_angles )
		# Create design and check validity
		self.design = SmoresDesign.SmoresDesign( root_module, self.module_list )
		self.design.check_validity();
		return

	def add_child_module(self, parent_module_name, new_module_name, module_params):
		pass


	def get_module_position(self, module_name):
		''' returns the current position of the middle of the module of interest, relative the root. '''
		assert self.name_mapping.has_key(module_name)
		module = self.name_mapping[module_name]
		# Get the KDL chain of the path from the root to the module of interest.
		# Note that node 1 is at the center of the module.
		(kdl_chain, joint_angles) = self.design.get_kinematics( self.root_node, module.nodes[1] )
		fk = ChainFKSolverPos_recursive(kdl_chain)
		final_frame = Frame()
		fk.JntToCart(joint_angles, final_frame)
		# return position and rotation matrix of final frame
		# TODO: make these into reasonable objects.
		return (finalFrame.p, finalFrame.M)




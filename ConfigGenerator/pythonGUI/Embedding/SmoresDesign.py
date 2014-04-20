'''
Created on Jan 15, 2014

@author: tariktosun
'''

from Design import *
import SmoresModule

class SmoresDesign(Design):
    '''
    SmoresDesign Class, which inherits from Design.
    '''


    def __init__(self, root_module, module_list):
        '''
        Constructor for SmoresDesign class.
        '''
        assert isinstance(root_module, SmoresModule.SmoresModule)
        
        # list of modules in this design:
        self.modules = module_list
        
        # generate a list of nodes in this design by concatenating node lists
        # from all modules:
        self.nodes = []
        for m in self.modules:
            self.nodes += m.nodes
        # remove duplicates from merging:
        self.nodes = list(set(self.nodes))
        
        # parse the underlying node tree:
        root_node = root_module.nodes[root_module.root_node_number]
        nodes = []
        edges= []
        self.parse_tree(root_node, nodes, edges)
        # ensure node_list and nodes have the same nodes:
        assert set(self.nodes) == set(nodes), 'Mismatch in tree and node_list'
        self.edges = edges
        self.root_node = root_node
        self.root_module = root_module
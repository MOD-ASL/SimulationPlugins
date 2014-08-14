"""
This class defines an entry of the SMORES Library.
Each entry includes one configuration and any number of gaits
that are associated with this configuration.
"""
class SmoresLibraryEntry(object):
    def __init__(self):
        self.entry_dir_path = '' # Directory in which the entry data is stored
        self.configuration_file_name = '' # File name of the configuration file
        self.gait_file_name_list = [] # A list of file name of the gaits file

    def __repr__(self):
        """
        Overwrite the print function of the class object
        """
        str_repr = ""
        # Only show the atrributes we are interested in
        key_list = ['entry_dir_path', 'gait_file_name_list']
        for key in key_list:
            if key == 'gait_file_name_list':
                str_repr = str_repr + \
                        ("{0:13}{1}\n".format("<"+key+">: ", ', '.join(\
                        getattr(self, key, []))))
            else:
                str_repr = str_repr + \
                        ("{0:13}{1}\n".format("<"+key+">: ", \
                        getattr(self, key, 'NOT DEFINED')))

        str_repr= "\n --SmoresLibraryEntry <{0}> -- \n".format(self.configuration_file_name) + \
                    str_repr + \
                    " -- End of SmoresLibraryEntry <{0}> -- \n".format(self.configuration_file_name)
        return str_repr

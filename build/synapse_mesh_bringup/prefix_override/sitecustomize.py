import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/akil/Desktop/synapse_mesh/install/synapse_mesh_bringup'

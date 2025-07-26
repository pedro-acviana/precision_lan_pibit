import py_trees
from px4_msgs.msg import VehicleCommand



class ajustar_posicao(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__('ajustar_posicao')


    def update(self):
        

        return py_trees.common.Status.SUCCESS

     
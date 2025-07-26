import py_trees
from px4_msgs.msg import VehicleCommand



class distancia_segura(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__('distancia_segura')


    def update(self):
        

        return py_trees.common.Status.SUCCESS

     
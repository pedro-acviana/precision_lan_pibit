import py_trees
from px4_msgs.msg import VehicleCommand
from precision_landing.nodos.deteccao import deteccao



class cor_detectada(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander, camera_detect):
        super().__init__('cor_detectada')


    def update(self):
        

        return py_trees.common.Status.FAILURE

class cor_detectada2(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander, camera_detect):
        super().__init__('cor_detectada')


    def update(self):
        

        return py_trees.common.Status.RUNNING


     
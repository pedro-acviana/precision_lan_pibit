import py_trees
from px4_msgs.msg import VehicleCommand



class CondicoesSeguras(py_trees.behaviour.Behaviour):
    def __init__(self, commander, camera_detect):
        super().__init__('Condicoes_seguras')
        self.camera = camera_detect
        self.cmdr = commander


    def update(self):

        if self.camera.camera_pronta:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
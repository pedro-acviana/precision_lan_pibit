import py_trees
from px4_msgs.msg import VehicleCommand



class OffboardMode(py_trees.behaviour.Behaviour):
    def __init__(self, commander):
        super().__init__('OffboardMode')
        self.cmdr = commander
        self.sent = False

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            self.cmdr.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.cmdr.get_logger().info("Enviando comando OFFBOARD")
            self.sent = True
            return py_trees.common.Status.RUNNING

        if self.sent:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING



     
import py_trees
from px4_msgs.msg import VehicleCommand



class Desarmar(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.cmdr = commander
        self.sent = False

    def update(self):
        if not self.sent:
            self.cmdr.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.cmdr.get_logger().info("Drone desarmado")
            self.sent = True
            return py_trees.common.Status.RUNNING

        if self.sent:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


     
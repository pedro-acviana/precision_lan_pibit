import py_trees
import time
from px4_msgs.msg import VehicleCommand



class Takeoff(py_trees.behaviour.Behaviour):
    def __init__(self, commander, altitude):
        super().__init__('TakeoffSetpoint')
        self.cmdr = commander
        self.alt = altitude
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if time.time() - self.start_time > 8.0:
            self.cmdr.get_logger().info("Voo estabilizado")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

     
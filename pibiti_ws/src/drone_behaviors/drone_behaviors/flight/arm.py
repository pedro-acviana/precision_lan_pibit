import py_trees
import time
from px4_msgs.msg import VehicleCommand



class Arm(py_trees.behaviour.Behaviour):
    def __init__(self, commander, camera_detect):
        super().__init__('Arm')
        self.cmdr = commander
        self.sent = False
        self.arm = False
        self.start_time = None
        self.camera = camera_detect

    def initialise(self):
        self.sent = False
        self.arm = False
        self.start_time = time.time()

    def update(self):
        self.cmdr.publish_offboard_control_heartbeat_signal()

        if not self.sent and self.cmdr.offboard_setpoint_counter >= 10:
            self.cmdr.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.cmdr.get_logger().info("Enviando comando de ARM")
            self.sent = True
            self.arm = True
            return py_trees.common.Status.RUNNING

        if self.arm:
            self.cmdr.get_logger().info("Drone armado")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

     
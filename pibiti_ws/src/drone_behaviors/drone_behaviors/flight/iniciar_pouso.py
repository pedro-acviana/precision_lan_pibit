import py_trees
from px4_msgs.msg import VehicleCommand
import time



class IniciarPouso(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.cmdr = commander
        self.sent = False
        self.start_time = None

    def initialise(self):
        self.sent = False
        self.start_time = time.time()

    def update(self):
        if hasattr(self.cmdr, 'landed_state') and self.cmdr.landed_state == 1:
            self.cmdr.get_logger().info("Drone already in landed state, no need to send LAND command")
            return py_trees.common.Status.SUCCESS
        # Envia o comando LAND uma vez
        if not self.sent:
            self.cmdr.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.cmdr.get_logger().info("Enviando comando LAND")
            self.sent = True
            return py_trees.common.Status.RUNNING

        # Calcula o tempo decorrido desde o envio do comando
        elapsed = time.time() - self.start_time

        # Verifica se pousou pelo estado ou por timeout
        if self.cmdr.landed_state == 1 or elapsed > 3.0:
            self.cmdr.get_logger().info(
                f" Pouso concluído (landed_state={self.cmdr.landed_state}, tempo={elapsed:.1f}s)"
            )
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

     
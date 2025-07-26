import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry 
import py_trees
import py_trees.display
import time


class PX4Commander(Node):
    def __init__(self):
        super().__init__('px4_pytrees_takeoff')
        qos = QoSProfile(depth=10)

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)
      
        self.arming_state = None
        self.nav_state = None
        self.landed_state = None


        self.target_altitude = 2.0
        self.clock = self.get_clock()

        self.offboard_setpoint_counter = 0

        # Envia setpoints continuamente para manter o modo OFFBOARD estável
        self.setpoint_timer = self.create_timer(0.05, self.publish_loop)

    def status_callback(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.landed_state = msg.landed_state
        self.get_logger().info(
            f"[STATUS] Arming: {msg.arming_state}, Nav: {msg.nav_state}, Landed: {msg.landed_state}"
        )

    def publish_loop(self):
        self.publish_offboard_control_heartbeat_signal()
        self.publish_setpoint(self.target_altitude)
        self.offboard_setpoint_counter += 1

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, z):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), -z]  # Usando posição Z alvo
        msg.velocity = [0.0, 0.0, 0.0]  # Velocidade de subida
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = float("nan")
        msg.timestamp = int(self.clock.now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.clock.now().nanoseconds / 1000)
        self.command_pub.publish(msg)


# --- BEHAVIOURS ---

class Arm(py_trees.behaviour.Behaviour):
    def __init__(self, commander):
        super().__init__('Arm')
        self.cmdr = commander
        self.sent = False
        self.arm = False
        self.start_time = None

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

class TakeoffSetpoint(py_trees.behaviour.Behaviour):
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


class Hover(py_trees.behaviour.Behaviour):
    def __init__(self, commander, duration=5.0):
        super().__init__('Hover')
        self.cmdr = commander
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()
        self.cmdr.get_logger().info(f"Drone mantendo hover por {self.duration} segundos.")

    def update(self):
        elapsed = time.time() - self.start_time

        # Envia o setpoint de posição atual para manter o hover
        self.cmdr.publish_setpoint(self.cmdr.target_altitude)

        if elapsed >= self.duration:
            self.cmdr.get_logger().info("Tempo de hover concluído.")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class Land(py_trees.behaviour.Behaviour):
    def __init__(self, commander):
        super().__init__('Land')
        self.cmdr = commander
        self.sent = False
        self.start_time = None

    def initialise(self):
        self.sent = False
        self.start_time = time.time()

    def update(self):
        # Envia o comando LAND uma vez
        if not self.sent:
            self.cmdr.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.cmdr.get_logger().info("Enviando comando LAND")
            self.sent = True
            return py_trees.common.Status.RUNNING

        # Calcula o tempo decorrido desde o envio do comando
        elapsed = time.time() - self.start_time

        # Verifica se pousou pelo estado ou por timeout
        if self.cmdr.landed_state == 1 or elapsed > 7.0:
            self.cmdr.get_logger().info(
                f"✅ Pouso concluído (landed_state={self.cmdr.landed_state}, tempo={elapsed:.1f}s)"
            )
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class Disarm(py_trees.behaviour.Behaviour):
    def __init__(self, commander):
        super().__init__('Disarm')
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


# --- ÁRVORE DE COMPORTAMENTO ---

def build_behavior_tree(commander):
    root = py_trees.composites.Sequence(name="Takeoff Mission", memory=True)
    root.add_children([
        Arm(commander),
        OffboardMode(commander),
        TakeoffSetpoint(commander, commander.target_altitude),
        Hover(commander, duration=5.0),
        Land(commander),
        Disarm(commander)
    ])
    return py_trees.trees.BehaviourTree(root)


# --- MAIN ---

def main(args=None):
    rclpy.init(args=args)
    commander = PX4Commander()
    tree = build_behavior_tree(commander)

    try:
        while rclpy.ok():
            rclpy.spin_once(commander, timeout_sec=0.1)
            tree.tick()
            print(py_trees.display.unicode_tree(tree.root, show_status=True))

            # Verifica se a árvore terminou (se o último comportamento foi concluído com sucesso)
            if tree.root.status == py_trees.common.Status.SUCCESS:
                commander.get_logger().info(" Missão completa.")
                break  # Finaliza a execução do ciclo principal

            time.sleep(0.05)  # Tick rate ~20Hz
    except KeyboardInterrupt:
        pass

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

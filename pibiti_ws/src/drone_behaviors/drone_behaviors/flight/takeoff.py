import py_trees
import time
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy



class Takeoff(py_trees.behaviour.Behaviour):
    def __init__(self, commander, altitude):
        super().__init__('TakeoffSetpoint')
        self.cmdr = commander
        self.alt = altitude
        self.start_time = 0.0  # Inicializa com valor padrão
        self.altitude_tolerance = 0.2  # Tolerância de 0.1 metros
        self.current_altitude = None  # Altitude obtida via subscriber
        
    def setup(self, **kwargs):
        # Initialize ROS2 components com o mesmo QoS do PX4Commander
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Adiciona o subscriber para a altitude atual
        self.cmdr.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )

        self.logger.info("Subscriber para altitude configurado com sucesso")
        return True
    
    def local_position_callback(self, msg):
        """Callback para atualizar a altitude atual"""
        self.current_altitude = msg.z

    def initialise(self):
        self.start_time = time.time()
        self.cmdr.get_logger().info(f"Iniciando takeoff para altitude alvo: {abs(self.cmdr.target_altitude):.2f}m")

    def update(self):
        # Timeout de segurança (30 segundos máximo)
        if time.time() - self.start_time > 50.0:
            self.cmdr.get_logger().warn("Timeout no takeoff - forçando sucesso")
            return py_trees.common.Status.FAILURE
        
        # Verifica se temos dados de altitude atual via subscriber
        if self.current_altitude is None:
            self.cmdr.get_logger().info("Aguardando dados de altitude do drone...")
            return py_trees.common.Status.RUNNING
        
        # Obtém altitude atual (valor absoluto, pois PX4 usa Z negativo)
        current_altitude = abs(self.current_altitude)
        target_altitude = abs(self.cmdr.target_altitude)
        
        # Calcula diferença entre altitude atual e alvo
        altitude_error = abs(current_altitude - target_altitude)
        
        # Log periódico do progresso
        elapsed_time = time.time() - self.start_time
        if int(elapsed_time) % 2 == 0 and elapsed_time - int(elapsed_time) < 0.1:  # A cada 2 segundos
            self.cmdr.get_logger().info(f"Takeoff: Alt atual: {current_altitude:.2f}m, Alvo: {target_altitude:.2f}m, Erro: {altitude_error:.2f}m")
        
        # Verifica se atingiu a altitude alvo dentro da tolerância
        if altitude_error <= self.altitude_tolerance:
            # Aguarda pelo menos 3 segundos para estabilização mesmo após atingir altitude
            if elapsed_time >= 3.0:
                self.cmdr.get_logger().info(f"Takeoff concluído! Altitude final: {current_altitude:.2f}m (erro: {altitude_error:.2f}m)")
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING

     
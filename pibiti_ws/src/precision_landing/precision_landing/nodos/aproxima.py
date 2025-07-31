import py_trees
import math
import numpy as np
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class aproxima(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.commander = commander
        self.tolerance = 0.5  # Tolerância em metros para considerar que chegou ao alvo
        self.max_velocity = 2.0  # Velocidade máxima em m/s
        self.landing_velocity = 0.5  # Velocidade de descida para pouso (m/s)
        self.target_reached = False
        self.landing_phase = False
        
        # Dados de posição atual do drone
        self.current_position = None
        self.target_absolute_position = None  # Posição absoluta do alvo (calculada uma vez)
        self.takeoff_altitude = None  # Altitude de takeoff para manter durante aproximação
        
        # Filtro de suavização
        self.last_velocity = np.array([0.0, 0.0, 0.0])
        self.smoothing_factor = 0.3  # Fator de suavização (0-1)
        
        # Subscrições ROS2
        self.position_subscriber = None
        
    def setup(self, **kwargs):
        """Configuração inicial com subscrições ROS2"""
        try:
            self.node = self.commander
            
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
            
            self.position_subscriber = self.node.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position',
                self.local_position_callback,
                qos_profile
            )
            
            self.logger.info("Aproxima: Subscrições ROS2 configuradas com sucesso")
            return True
            
        except Exception as e:
            self.logger.error(f"Erro na configuração do aproxima: {e}")
            return False
    
    def local_position_callback(self, msg):
        """Callback para receber posição local do drone"""
        self.current_position = {
            'x': msg.y,  # X = Lateral 
            'y': msg.x,  # Y = Longitudinal)
            'z': msg.z   # Vertical 
        }
    
    def initialise(self):
        """Inicializa a aproximação calculando o alvo absoluto uma vez"""
        self.logger.info("Iniciando aproximação ao local seguro")
        self.target_reached = False
        self.landing_phase = False
        self.last_velocity = np.array([0.0, 0.0, 0.0])
        
        # Calcula o alvo absoluto
        try:
            blackboard = py_trees.blackboard.Blackboard()
            if not blackboard.exists("target_relative_position"):
                self.logger.error("Posição relativa do alvo não encontrada")
                self.target_absolute_position = None
                return
                
            target_relative = blackboard.get("target_relative_position")
            
            if self.current_position is None:
                self.logger.warning("Aguardando posição atual do drone...")
                self.target_absolute_position = None
                return
                
            # Calcula posição absoluta do alvo (FIXA)
            self.target_absolute_position = {
                'x': self.current_position['x'] + target_relative['x'],
                'y': self.current_position['y'] + target_relative['y'],
                'z': self.current_position['z']  # Mantém altitude atual
            }
            
            # Salva altitude de takeoff para manter durante aproximação
            self.takeoff_altitude = self.current_position['z']
            
            self.logger.info(f"Alvo absoluto calculado: x={self.target_absolute_position['x']:.2f}, y={self.target_absolute_position['y']:.2f}")
            self.logger.info(f"Altitude de takeoff salva: z={self.takeoff_altitude:.2f}")
            
        except Exception as e:
            self.logger.error(f"Erro na inicialização: {e}")
            self.target_absolute_position = None
    
    def smooth_velocity(self, desired_velocity):
        """Aplica suavização exponencial à velocidade"""
        smoothed = (1 - self.smoothing_factor) * self.last_velocity + \
                  self.smoothing_factor * np.array(desired_velocity)
        self.last_velocity = smoothed
        return smoothed.tolist()
    
    def calculate_adaptive_velocity(self, distance, altitude):
        """Calcula velocidade adaptativa baseada na distância e altitude"""
        # Velocidade proporcional à distância, limitada pela altitude
        base_speed = min(self.max_velocity, distance * 0.5)
        
        # Reduz velocidade quando mais próximo do solo
        if altitude < 5.0:  # Abaixo de 5m
            altitude_factor = max(0.2, altitude / 5.0)  # Não menos que 20% da velocidade
            base_speed *= altitude_factor
        
        return base_speed
        
    def update(self):
        """Controle baseado no alvo absoluto fixo calculado no initialise"""
        try:
            if self.current_position is None:
                self.logger.warning("Aguardando posição atual do drone...")
                return py_trees.common.Status.RUNNING
                
            if self.target_absolute_position is None:
                self.logger.warning("Alvo absoluto não calculado ainda...")
                return py_trees.common.Status.RUNNING
                
            if self.takeoff_altitude is None:
                self.logger.warning("Altitude de takeoff não salva ainda...")
                return py_trees.common.Status.RUNNING

            # Calcula o erro como diferença entre alvo absoluto (fixo) e posição atual
            error_x = self.target_absolute_position['x'] - self.current_position['x'] 
            error_y = self.target_absolute_position['y'] - self.current_position['y']
            error_z = self.takeoff_altitude - self.current_position['z']  # Erro de altitude
            distance_horizontal = math.sqrt(error_x**2 + error_y**2)
            
            # Controle proporcional baseado no erro
            kp = 1.0  # Ganho proporcional
            kp_z = 0.5  # Ganho proporcional para altitude (mais suave)
            
            # Calcula velocidades proporcionais ao erro (sempre ativas)
            velocity_x = kp * error_x  
            velocity_y = kp * error_y
            
            # Calcula altitude atual para usar nas condições
            current_altitude = abs(self.current_position['z'])
            
            # Aplica velocidade adaptativa baseada na distância e altitude
            adaptive_speed = self.calculate_adaptive_velocity(distance_horizontal, current_altitude)
            
            # Normaliza e escala as velocidades horizontais (sempre)
            if distance_horizontal > 0.01:  # Evita divisão por zero
                scale_factor = min(1.0, adaptive_speed / distance_horizontal)
                velocity_x *= scale_factor
                velocity_y *= scale_factor
            
            # Controle de altitude depende da fase
            if distance_horizontal < self.tolerance:
                if not self.landing_phase:
                    self.landing_phase = True
                    self.logger.warning("Alvo alcançado! Iniciando fase de pouso preciso!")
                
                # Durante o pouso: mantém controle horizontal, mas desce
                velocity_z = self.landing_velocity  # Velocidade constante de descida
                
                # Verifica se pousou (altitude muito baixa)
                if current_altitude < 0.3:
                    self.logger.info("Pouso concluído!")
                    self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
                    return py_trees.common.Status.SUCCESS
            else:
                # Durante aproximação: mantém altitude de takeoff
                velocity_z = kp_z * error_z

            # Limita as velocidades máximas
            velocity_x = np.clip(velocity_x, -self.max_velocity, self.max_velocity)
            velocity_y = np.clip(velocity_y, -self.max_velocity, self.max_velocity)
            velocity_z = np.clip(velocity_z, -self.max_velocity, self.max_velocity)

            # Aplica suavização às velocidades (incluindo Z)
            smoothed_velocity = self.smooth_velocity([velocity_x, velocity_y, velocity_z])
            velocity_x, velocity_y, velocity_z = smoothed_velocity[0], smoothed_velocity[1], smoothed_velocity[2]

            self.commander.publish_velocity_setpoint(velocity_y, velocity_x, velocity_z)

            # Log detalhado para debug - apenas de vez em quando para não poluir
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
                
            if self._log_counter % 20 == 0:  # A cada 20 iterações (aproximadamente 1 segundo)
                self.logger.info(f"Pos atual: X={self.current_position['x']:.2f}, Y={self.current_position['y']:.2f}, Z={self.current_position['z']:.2f}")
                self.logger.info(f"Alvo: X={self.target_absolute_position['x']:.2f}, Y={self.target_absolute_position['y']:.2f}, Z_takeoff={self.takeoff_altitude:.2f}")
                self.logger.info(f"Erro: X={error_x:.2f}m, Y={error_y:.2f}m, Z={error_z:.2f}m, Dist={distance_horizontal:.2f}m")
                self.logger.info(f"Vel cmd: Vx={velocity_x:.2f}, Vy={velocity_y:.2f}, Vz={velocity_z:.2f}")

            return py_trees.common.Status.RUNNING

        except Exception as e:
            self.logger.error(f"Erro no controle: {e}")
            self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """Limpeza final"""
        self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
        
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Pouso concluído com sucesso")
        else:
            self.logger.info("Aproximação finalizada")
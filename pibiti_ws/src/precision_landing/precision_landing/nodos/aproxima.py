import py_trees
import math
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
        # Posição absoluta do alvo (calculada uma vez no início)
        self.target_absolute_position = None
        
        # Subscrições ROS2
        self.position_subscriber = None
        
    def setup(self, **kwargs):
        """Configuração inicial com subscrições ROS2"""
        try:
            # Usar o commander como node para as subscrições
            self.node = self.commander
            
            # Configurar QoS igual ao PX4Commander
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
            
            # Subscrever à posição local do veículo
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
            'x': msg.x,  # Norte (NED)
            'y': msg.y,  # Leste (NED)
            'z': msg.z   # Baixo (NED)
        }
    def initialise(self):
        """Inicializa a aproximação calculando a posição absoluta do alvo uma vez"""
        self.logger.info("Iniciando aproximação ao local seguro usando coordenadas relativas")
        self.target_reached = False
        self.landing_phase = False
        
        try:
            # Obtém as coordenadas relativas do blackboard
            blackboard = py_trees.blackboard.Blackboard()
            if not blackboard.exists("target_relative_position"):
                self.logger.error("Posição relativa do alvo não encontrada no blackboard")
                return
                
            target_relative = blackboard.get("target_relative_position")
            
            # Aguarda ter a posição atual do drone
            if self.current_position is None:
                self.logger.warning("Aguardando posição atual do drone...")
                return
                
            # Calcula posição absoluta do alvo (uma vez só no início)
            self.target_absolute_position = {
                'x': self.current_position['x'] + target_relative['x'],
                'y': self.current_position['y'] + target_relative['y'],
                'z': self.current_position['z']  # Mantém altitude atual
            }
            
            self.logger.info(f"Alvo absoluto calculado: x={self.target_absolute_position['x']:.2f}, y={self.target_absolute_position['y']:.2f}")
            
        except Exception as e:
            self.logger.error(f"Erro na inicialização da aproximação: {e}")
        
    def update(self):
        """
        Move o drone calculando distância restante até o alvo e executa pouso preciso
        """
        try:
            # Verifica se tem posição atual do drone
            if self.current_position is None:
                self.logger.warning("Aguardando dados de posição do drone...")
                return py_trees.common.Status.RUNNING
            
            # Verifica se o alvo absoluto foi calculado
            if self.target_absolute_position is None:
                self.logger.warning("Alvo absoluto não calculado ainda...")
                return py_trees.common.Status.RUNNING
            
            # Calcula distância ATUAL (restante) até o alvo absoluto
            distance_x = self.target_absolute_position['x'] - self.current_position['x']
            distance_y = self.target_absolute_position['y'] - self.current_position['y']
            horizontal_distance = math.sqrt(distance_x**2 + distance_y**2)
            
            # Verifica se chegou na posição horizontal (tolerância)
            if horizontal_distance < self.tolerance:
                if not self.landing_phase:
                    self.landing_phase = True
                    self.logger.info(f"Posição horizontal alcançada! Iniciando pouso preciso...")
                
                # Verifica se o drone está muito baixo (menor que 50cm) - deve pousar
                current_altitude = abs(self.current_position['z'])  # Altitude positiva
                if current_altitude < 0.5:  # Menor que 50cm
                    self.logger.info(f"Altitude crítica atingida ({current_altitude:.2f}m) - finalizando pouso")
                    # Para o movimento e finaliza com sucesso
                    self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
                    return py_trees.common.Status.SUCCESS
                
                # Fase de pouso: pequenas correções e descida controlada
                correction_x = -distance_x * 0.3  # Correção suave baseada na distância restante
                correction_y = -distance_y * 0.3
                
                # Limita correções para evitar movimentos bruscos
                correction_x = max(-0.2, min(0.2, correction_x))
                correction_y = max(-0.2, min(0.2, correction_y))
                
                # Comando de pouso com correção de posição
                self.commander.publish_velocity_setpoint(
                    correction_x,           # Pequena correção Norte
                    correction_y,           # Pequena correção Leste  
                    self.landing_velocity   # Descida controlada
                )
                
                self.logger.info(f"Pousando: alt={current_altitude:.2f}m, dist_restante={horizontal_distance:.3f}m, corr=({correction_x:.3f}, {correction_y:.3f})")
                
                return py_trees.common.Status.RUNNING
                
            else:
                # Fase de aproximação: calcula velocidades baseadas na distância restante
                if horizontal_distance > 0:
                    # Normaliza direção e aplica velocidade baseada na distância
                    velocity_factor = min(self.max_velocity, horizontal_distance) / horizontal_distance
                    velocity_x = distance_x * velocity_factor
                    velocity_y = distance_y * velocity_factor
                else:
                    velocity_x = 0.0
                    velocity_y = 0.0
                
                # Envia comando de velocidade (mantém altitude)
                self.commander.publish_velocity_setpoint(
                    velocity_x,  # Velocidade Norte (baseada na distância restante)
                    velocity_y,  # Velocidade Leste (baseada na distância restante)
                    0.0          # Mantém altitude
                )
                
                self.logger.info(f"Aproximando: dist_restante={horizontal_distance:.2f}m, vel=({velocity_x:.2f}, {velocity_y:.2f})")
                
                return py_trees.common.Status.RUNNING
            
        except Exception as e:
            self.logger.error(f"Erro durante aproximação: {e}")
            # Para o drone em caso de erro
            self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """Para o drone quando o comportamento termina"""
        # Para o drone
        self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
        
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Aproximação e pouso concluídos com sucesso")
        else:
            self.logger.info("Aproximação ao local seguro finalizada")
        
        # Limpa subscrições se necessário
        if hasattr(self, 'position_subscriber') and self.position_subscriber:
            try:
                self.node.destroy_subscription(self.position_subscriber)
            except:
                pass

     
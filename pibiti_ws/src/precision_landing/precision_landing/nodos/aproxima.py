import py_trees
import math
import time
import numpy as np
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from precision_landing.utils.kalman_filter import PositionKalmanFilter


class PIController:
    """
    Controlador Proporcional-Integral para controle de posição
    Baseado nos conceitos de controle do Capítulo 4 do livro de Corke
    """
    
    def __init__(self, kp=1.0, ki=0.1, windup_limit=5.0):
        """
        Inicializa o controlador PI
        
        Args:
            kp: Ganho proporcional
            ki: Ganho integral
            windup_limit: Limite para anti-windup
        """
        self.kp = kp
        self.ki = ki
        self.windup_limit = windup_limit
        
        # Estado interno
        self.integral_error = np.array([0.0, 0.0, 0.0])
        self.last_time = None
        
    def compute(self, error, dt):
        """
        Calcula a saída do controlador PI
        
        Args:
            error: Erro atual [x, y, z]
            dt: Delta de tempo
            
        Returns:
            output: Comando de velocidade [vx, vy, vz]
        """
        # Atualiza erro integral com anti-windup
        self.integral_error += error * dt
        
        # Aplica limite anti-windup
        self.integral_error = np.clip(self.integral_error, 
                                    -self.windup_limit, 
                                    self.windup_limit)
        
        # Calcula saída PI
        output = self.kp * error + self.ki * self.integral_error
        
        return output
    
    def reset(self):
        """
        Reseta o estado interno do controlador
        """
        self.integral_error = np.array([0.0, 0.0, 0.0])


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
        
        # Controladores PI para cada eixo
        self.controller_x = PIController(kp=1.2, ki=0.2, windup_limit=3.0)
        self.controller_y = PIController(kp=1.2, ki=0.2, windup_limit=3.0)
        self.controller_z = PIController(kp=0.8, ki=0.1, windup_limit=2.0)
        
        # Filtro de Kalman para estimativa de posição/velocidade
        self.position_filter = PositionKalmanFilter(
            process_noise=0.1,
            measurement_noise_vision=0.3,
            measurement_noise_imu=0.1
        )
        
        # Filtro de suavização
        self.last_velocity = np.array([0.0, 0.0, 0.0])
        self.smoothing_factor = 0.3  # Fator de suavização (0-1)
        self.last_control_time = None
        
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
        
        # Atualiza filtro de Kalman com medição de posição
        current_pos = [self.current_position['x'], self.current_position['y']]
        self.position_filter.predict()
        self.position_filter.update_vision(current_pos)
    
    def initialise(self):
        """Inicializa a aproximação calculando o alvo absoluto uma vez"""
        self.logger.info("Iniciando aproximação ao local seguro com controle PI avançado")
        self.target_reached = False
        self.landing_phase = False
        self.last_velocity = np.array([0.0, 0.0, 0.0])
        self.last_control_time = None
        
        # Reseta controladores PI
        self.controller_x.reset()
        self.controller_y.reset()
        self.controller_z.reset()
        
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
    
    def calculate_adaptive_velocity(self, distance, altitude, current_velocity):
        """
        Calcula velocidade adaptativa baseada na distância, altitude e velocidade atual
        Incorpora informações do filtro de Kalman para controle mais suave
        """
        # Velocidade base proporcional à distância
        base_speed = min(self.max_velocity, distance * 0.8)
        
        # Reduz velocidade quando mais próximo do solo
        if altitude < 5.0:  # Abaixo de 5m
            altitude_factor = max(0.3, altitude / 5.0)  # Não menos que 30% da velocidade
            base_speed *= altitude_factor
        
        # Considera velocidade atual para evitar mudanças bruscas
        if current_velocity is not None:
            current_speed = np.linalg.norm(current_velocity[:2])  # Velocidade horizontal
            max_accel = 2.0  # m/s²
            dt = 0.05  # Tempo de ciclo aproximado
            max_speed_change = max_accel * dt
            
            if abs(base_speed - current_speed) > max_speed_change:
                if base_speed > current_speed:
                    base_speed = current_speed + max_speed_change
                else:
                    base_speed = max(0, current_speed - max_speed_change)
        
        return base_speed
        
    def update(self):
        """Controle baseado em controladores PI e filtros de Kalman"""
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

            # Calcula delta de tempo para controladores
            current_time = time.time()
            if self.last_control_time is None:
                dt = 0.05  # Valor inicial
            else:
                dt = current_time - self.last_control_time
            self.last_control_time = current_time

            # Calcula erros horizontais
            error_x = self.target_absolute_position['x'] - self.current_position['x'] 
            error_y = self.target_absolute_position['y'] - self.current_position['y']
            distance_horizontal = math.sqrt(error_x**2 + error_y**2)
            
            # Obtém estimativas do filtro de Kalman
            position_estimate = self.position_filter.get_position_estimate()
            velocity_estimate = self.position_filter.get_velocity_estimate()
            
            # Calcula altitude atual para condições
            current_altitude = abs(self.current_position['z'])
            
            # Durante aproximação vs. durante pouso
            if distance_horizontal < self.tolerance:
                if not self.landing_phase:
                    self.landing_phase = True
                    self.logger.warning("Alvo alcançado! Iniciando fase de pouso preciso!")
                
                # Durante o pouso: controle horizontal PI + descida suave para altitude zero
                velocity_x = self.controller_x.compute(np.array([error_x]), dt)[0]
                velocity_y = self.controller_y.compute(np.array([error_y]), dt)[0]
                velocity_xy = np.array([velocity_x, velocity_y])
                
                # Para pouso: erro Z é sempre a altitude atual (queremos chegar a zero)
                error_z = 0.0 - self.current_position['z']  # Alvo é altitude zero
                velocity_z = self.landing_velocity  # Velocidade constante de descida
                
                # Verifica se pousou
                if current_altitude < 0.3:
                    self.logger.info("Pouso concluído!")
                    self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
                    return py_trees.common.Status.SUCCESS
            else:
                # Durante aproximação: controle PI completo para manter altitude de takeoff
                error_z = self.takeoff_altitude - self.current_position['z']  # Mantém altitude de takeoff
                velocity_x = self.controller_x.compute(np.array([error_x]), dt)[0]
                velocity_y = self.controller_y.compute(np.array([error_y]), dt)[0]
                velocity_z = self.controller_z.compute(np.array([error_z]), dt)[0]
                
                velocity_xy = np.array([velocity_x, velocity_y])

            # Aplica limitação de velocidade adaptativa
            adaptive_speed = self.calculate_adaptive_velocity(
                distance_horizontal, current_altitude, velocity_estimate
            )
            
            # Normaliza e escala as velocidades horizontais
            if distance_horizontal > 0.01:
                current_speed = np.linalg.norm(velocity_xy)
                if current_speed > adaptive_speed:
                    scale_factor = adaptive_speed / current_speed
                    velocity_xy *= scale_factor

            # Limita velocidades máximas
            velocity_x = np.clip(velocity_xy[0], -self.max_velocity, self.max_velocity)
            velocity_y = np.clip(velocity_xy[1], -self.max_velocity, self.max_velocity)
            velocity_z = np.clip(velocity_z, -self.max_velocity, self.max_velocity)

            # Aplica suavização às velocidades
            smoothed_velocity = self.smooth_velocity([velocity_x, velocity_y, velocity_z])
            velocity_x, velocity_y, velocity_z = smoothed_velocity

            # Publica comando de velocidade (lembra da conversão de coordenadas)
            self.commander.publish_velocity_setpoint(velocity_y, velocity_x, velocity_z)

            # Log detalhado para debug - apenas ocasionalmente
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
                
            if self._log_counter % 20 == 0:  # A cada 20 iterações
                phase_str = "POUSO" if self.landing_phase else "APROXIMAÇÃO"
                self.logger.info(f"[{phase_str}] Pos atual: X={self.current_position['x']:.2f}, Y={self.current_position['y']:.2f}, Z={self.current_position['z']:.2f}")
                self.logger.info(f"[{phase_str}] Alvo: X={self.target_absolute_position['x']:.2f}, Y={self.target_absolute_position['y']:.2f}")
                self.logger.info(f"[{phase_str}] Erro: X={error_x:.2f}m, Y={error_y:.2f}m, Z={error_z:.2f}m, Dist={distance_horizontal:.2f}m")
                self.logger.info(f"[{phase_str}] Vel estimada KF: vx={velocity_estimate[0]:.2f}, vy={velocity_estimate[1]:.2f} m/s")
                self.logger.info(f"[{phase_str}] Vel cmd: Vx={velocity_x:.2f}, Vy={velocity_y:.2f}, Vz={velocity_z:.2f}")

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
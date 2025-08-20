import py_trees
import numpy as np
import math
import pickle
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleImuStatus
from precision_landing.utils.kalman_filter import DepthEstimationKalmanFilter, PositionKalmanFilter, TemplateTracker
from precision_landing.utils.enhanced_pose_estimation import EnhancedPose2D3D


class img2local(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        
        # Parâmetros intrínsecos da câmera (RaspCam)
        self.fx = 540  # focal length x (pixels)
        self.fy = 627  # focal length y (pixels)
        self.cx = 640  # Centro X = 1280/2
        self.cy = 480  # Centro Y = 960/2
        
        # Dimensões da imagem
        self.image_width = 1280   # Largura da imagem em pixels
        self.image_height = 960   # Altura da imagem em pixels

        # Matriz de calibração intrínseca K
        self.K = np.array([
            [self.fx, 0,       self.cx],
            [0,       self.fy, self.cy],
            [0,       0,       1]
        ])
        
        # Inicializa sistemas avançados de pose estimation
        self.pose_estimator = EnhancedPose2D3D(
            camera_matrix=self.K,
            image_size=(self.image_width, self.image_height)
        )
        
        # Filtros de Kalman
        self.depth_filter = DepthEstimationKalmanFilter(
            initial_depth=5.0,
            process_noise=0.1,
            measurement_noise=0.5
        )
        
        self.position_filter = PositionKalmanFilter(
            process_noise=0.1,
            measurement_noise_vision=0.5,
            measurement_noise_imu=0.2
        )
        
        # Rastreador de homografia
        self.template_tracker = TemplateTracker()
        
        # Inicializa variáveis
        self.current_altitude = None
        self.current_velocity = None
        self.node = None
        
        # Histórico para SfM
        self.feature_history = []
        self.reference_features_set = False
        
    def setup(self, **kwargs):
        if 'node' not in kwargs:
            self.logger.warning("Parâmetro 'node' não fornecido - subscriber não será criado")
            return True
            
        self.node = kwargs['node']
        
        # Initialize ROS2 components
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscriber para posição local
        self.node.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )
        
        # Subscriber para dados da IMU (velocidade)
        self.node.create_subscription(
            VehicleImuStatus,
            '/fmu/out/vehicle_imu_status',
            self.imu_callback,
            qos_profile
        )

        return True
    
    def local_position_callback(self, msg):
        """Callback para atualizar a altitude atual"""
        self.current_altitude = -msg.z  # Inverte Z para ter altitude positiva
        
        # Atualiza filtro de posição com dados visuais quando disponível
        if hasattr(self, 'last_visual_position'):
            self.position_filter.predict()
            self.position_filter.update_position(self.last_visual_position)
    
    def imu_callback(self, msg):
        """Callback para dados da IMU"""
        # Extrai velocidade da IMU se disponível
        # (Nota: VehicleImuStatus pode não ter velocidade direta)
        pass
        
    def initialise(self):
        self.logger.info("Iniciando conversão de coordenadas pixel para coordenadas relativas")
    
    def enhanced_pixel_to_relative_position(self, u, v, altitude, current_features=None):
        """
        Conversão melhorada usando pose estimation e filtros de Kalman com TODAS as velocidades
        """
        try:
            # Atualiza filtros de Kalman
            self.depth_filter.predict()
            self.position_filter.predict()
            
            # === USAR VELOCIDADES DO TEMPLATE TRACKER ===
            template_velocity = None
            area_change_rate = None
            if self.template_tracker.template is not None:
                # Obtém velocidades do template (pixels/segundo)
                template_velocity = self.template_tracker.get_velocity_estimate()
                area_change_rate = self.template_tracker.get_area_change_rate()
                
                # Log das velocidades do template
                if template_velocity != (0.0, 0.0):
                    self.logger.info(f"Template velocity: vx={template_velocity[0]:.2f}, vy={template_velocity[1]:.2f} px/s")
                
                # Detecção de comportamento de zoom
                if area_change_rate > 100:
                    self.logger.info("Drone se aproximando rapidamente (área aumentando)")
                elif area_change_rate < -100:
                    self.logger.info("Drone se afastando (área diminuindo)")
            
            # === COMPENSAÇÃO DE MOVIMENTO DE CÂMERA ===
            u_compensated, v_compensated = u, v
            if template_velocity and template_velocity != (0.0, 0.0):
                # Compensa movimento aparente do template
                compensation_factor = 0.1  # Fator de compensação
                u_compensated = u - template_velocity[0] * compensation_factor
                v_compensated = v - template_velocity[1] * compensation_factor
                
                self.logger.debug(f"Compensação aplicada: du={-template_velocity[0] * compensation_factor:.2f}, dv={-template_velocity[1] * compensation_factor:.2f}")
            
            # Estima profundidade usando Structure-from-Motion se temos features
            depth_estimate = None
            if current_features and len(self.feature_history) > 0:
                depth_estimate = self.pose_estimator.estimate_depth_from_apparent_size(
                    current_features, self.feature_history[-1]
                )
                
                if depth_estimate:
                    self.depth_filter.update(depth_estimate)
                    depth_estimate = self.depth_filter.get_depth_estimate()
                    
                    # === USAR VELOCIDADE DE PROFUNDIDADE ===
                    depth_velocity = self.depth_filter.state[1]  # Velocidade de profundidade
                    if abs(depth_velocity) > 0.1:
                        self.logger.info(f"Velocidade de profundidade: {depth_velocity:.2f} m/s")
                        
                        # Predição de profundidade futura
                        prediction_time = 0.5  # 500ms no futuro
                        predicted_depth = depth_estimate + depth_velocity * prediction_time
                        self.logger.info(f"Profundidade predita (500ms): {predicted_depth:.2f}m")
            
            # Usa conversão melhorada com coordenadas compensadas
            x, y, z = self.pose_estimator.enhanced_pixel_to_3d(
                u_compensated, v_compensated, altitude, depth_estimate, current_features
            )
            
            # Valida resultado
            if not self.pose_estimator.validate_3d_coordinates(x, y, z):
                self.logger.warning("Coordenadas 3D inválidas, usando método tradicional")
                return self.pixel_to_relative_position_fallback(u, v, altitude)
            
            # Atualiza filtro de posição
            self.position_filter.update_position([x, y])
            smoothed_position = self.position_filter.get_position_estimate()
            
            # === USAR VELOCIDADE DE POSIÇÃO PARA PREDIÇÃO ===
            position_velocity = self.position_filter.get_velocity_estimate()
            if position_velocity is not None and np.linalg.norm(position_velocity) > 0.1:
                self.logger.info(f"Velocidade de posição: vx={position_velocity[0]:.2f}, vy={position_velocity[1]:.2f} m/s")
                
                # Predição de posição futura
                prediction_time = 0.5
                predicted_x = smoothed_position[0] + position_velocity[0] * prediction_time
                predicted_y = smoothed_position[1] + position_velocity[1] * prediction_time
                
                self.logger.info(f"Posição predita (500ms): x={predicted_x:.2f}, y={predicted_y:.2f}m")
                
                # Salva predição no blackboard para uso pelo aproxima.py
                blackboard = py_trees.blackboard.Blackboard()
                blackboard.set("predicted_target_position", {
                    'x': predicted_x,
                    'y': predicted_y,
                    'prediction_time': prediction_time
                })
            
            # === DETECÇÃO DE ESTABILIDADE ===
            is_target_stable = self._check_target_stability(
                position_velocity, template_velocity, area_change_rate
            )
            
            # Salva features atuais para próxima iteração
            if current_features:
                self.feature_history.append(current_features)
                if len(self.feature_history) > 5:  # Mantém apenas 5 frames
                    self.feature_history.pop(0)
            
            return np.array([smoothed_position[0], smoothed_position[1], 0])
            
        except Exception as e:
            self.logger.error(f"Erro na conversão melhorada: {e}")
            return self.pixel_to_relative_position_fallback(u, v, altitude)
    
    def _check_target_stability(self, position_velocity, template_velocity, area_change_rate):
        """
        Verifica se o alvo está estável baseado em TODAS as velocidades
        
        Args:
            position_velocity: Velocidade de posição do Kalman Filter
            template_velocity: Velocidade do template tracker
            area_change_rate: Taxa de mudança da área
            
        Returns:
            bool: True se o alvo está estável
        """
        # Limites de estabilidade
        position_threshold = 0.2  # m/s
        template_threshold = 5.0  # pixels/s
        area_threshold = 50.0     # pixels²/s
        
        # Verifica velocidade de posição
        position_stable = True
        if position_velocity is not None:
            position_speed = np.linalg.norm(position_velocity)
            position_stable = position_speed < position_threshold
        
        # Verifica velocidade do template
        template_stable = True
        if template_velocity and template_velocity != (0.0, 0.0):
            template_speed = np.sqrt(template_velocity[0]**2 + template_velocity[1]**2)
            template_stable = template_speed < template_threshold
        
        # Verifica mudança de área
        area_stable = True
        if area_change_rate is not None:
            area_stable = abs(area_change_rate) < area_threshold
        
        is_stable = position_stable and template_stable and area_stable
        
        # Log do status de estabilidade
        if not is_stable:
            reasons = []
            if not position_stable:
                reasons.append(f"posição instável ({np.linalg.norm(position_velocity):.2f} m/s)")
            if not template_stable:
                template_speed = np.sqrt(template_velocity[0]**2 + template_velocity[1]**2)
                reasons.append(f"template instável ({template_speed:.2f} px/s)")
            if not area_stable:
                reasons.append(f"área instável ({area_change_rate:.1f} px²/s)")
            
            self.logger.warning(f"Alvo instável: {', '.join(reasons)}")
        
        # Salva status no blackboard
        blackboard = py_trees.blackboard.Blackboard()
        blackboard.set("target_stable", is_stable)
        
        return is_stable
    
    def pixel_to_relative_position_fallback(self, u, v, altitude):
        """
        Método de fallback usando a conversão tradicional
        """
        try:
            # Converte pixel para coordenadas normalizadas da câmera
            x_normalized = (u - self.cx) / self.fx
            y_normalized = (v - self.cy) / self.fy
            
            # Para câmera apontando para frente, verifica se pixel está abaixo do horizonte
            if v <= self.cy:
                self.logger.warning(f"Pixel ({u}, {v}) está no horizonte ou acima")
                return np.array([0, 0, 0])
            
            # Calcula ângulos
            angle_depression = math.atan(y_normalized)
            angle_lateral = math.atan(x_normalized)
            
            if angle_depression <= 0:
                self.logger.warning("Ângulo de depressão inválido")
                return np.array([0, 0, 0])
                
            # Trigonometria básica
            distance_forward = altitude / math.tan(angle_depression)
            distance_lateral = distance_forward * math.tan(angle_lateral)
            
            return np.array([distance_forward, -distance_lateral, 0])
            
        except Exception as e:
            self.logger.error(f"Erro no método de fallback: {e}")
            return np.array([0, 0, 0])
    
    def update(self):
        try:
            blackboard = py_trees.blackboard.Blackboard()
            
            # Verifica se o local seguro foi determinado após estabilização
            if not blackboard.exists("local_seguro_pixel"):
                return py_trees.common.Status.RUNNING
                
            # Verifica se há score (indica que a estabilização foi concluída)
            if not blackboard.exists("local_seguro_score"):
                self.logger.warning("Aguardando conclusão da fase de estabilização...")
                return py_trees.common.Status.RUNNING
                
            pixel_pos = blackboard.get("local_seguro_pixel")
            pixel_x, pixel_y = pixel_pos
            
            # Validação das coordenadas
            pixel_x = max(0, min(pixel_x, self.image_width - 1))
            pixel_y = max(0, min(pixel_y, self.image_height - 1))
            
            if self.current_altitude is None:
                self.logger.warning("Posição atual do drone não disponível")
                return py_trees.common.Status.RUNNING
                
            altitude = abs(self.current_altitude)
            if altitude < 0.1:
                self.logger.warning("Altitude muito baixa - usando valor mínimo")
                altitude = 1.0
            
            # === INTEGRAÇÃO COM TEMPLATE TRACKER ===
            # Configura template tracker se ainda não foi feito
            if self.template_tracker.template is None and blackboard.exists("current_image"):
                current_image = blackboard.get("current_image")
                success = self.template_tracker.set_reference_template(
                    current_image, pixel_x, pixel_y, region_size=40
                )
                if success:
                    self.logger.info("Template de referência configurado com sucesso")
                else:
                    self.logger.warning("Falha ao configurar template de referência")
            
            # Rastreia template na imagem atual para obter velocidades
            current_features = None
            if blackboard.exists("current_image") and self.template_tracker.template is not None:
                current_image = blackboard.get("current_image")
                tracking_result = self.template_tracker.track_template(current_image)
                
                if tracking_result['found'] and tracking_result['confidence'] > 0.7:
                    # Usa posição rastreada pelo template (mais precisa)
                    tracked_x, tracked_y = tracking_result['position']
                    pixel_x, pixel_y = tracked_x, tracked_y
                    
                    # Usa área do template como feature para SfM
                    current_features = tracking_result['area_ratio']
                    
                    self.logger.info(f"Template rastreado: confiança={tracking_result['confidence']:.2f}, "
                                   f"área_ratio={tracking_result['area_ratio']:.2f}")
                    
                    # Salva informações de tracking no blackboard
                    blackboard.set("template_tracking_result", tracking_result)
                else:
                    self.logger.warning("Template tracking falhou - usando posição original")
            
            # Tenta obter features atuais para SfM (fallback se template não funcionar)
            if current_features is None and blackboard.exists("current_features"):
                current_features = blackboard.get("current_features")
            
            # Usa o método melhorado de conversão com TODAS as velocidades
            relative_pos = self.enhanced_pixel_to_relative_position(
                pixel_x, pixel_y, altitude, current_features
            )
            
            relative_target = {
                'x': relative_pos[0],  # X = Longitudinal (frente/trás)
                'y': relative_pos[1],  # Y = Lateral (esquerda/direita, + esquerda, - direita) 
                'z': 0                 # Z = Altitude mantida
            }
            
            # Salva no blackboard
            blackboard.set("target_relative_position", relative_target)
            
            # Salva última posição visual para o filtro
            self.last_visual_position = [relative_target['x'], relative_target['y']]
            
            # Obtém estimativas dos filtros para logging completo
            depth_estimate = self.depth_filter.get_depth_estimate()
            depth_uncertainty = self.depth_filter.get_depth_uncertainty()
            depth_velocity = self.depth_filter.state[1]  # Velocidade de profundidade
            
            position_estimate = self.position_filter.get_position_estimate()
            velocity_estimate = self.position_filter.get_velocity_estimate()
            
            # Obtém velocidades do template tracker
            template_velocity = self.template_tracker.get_velocity_estimate()
            area_change_rate = self.template_tracker.get_area_change_rate()
            
            score = blackboard.get("local_seguro_score")
            
            # Log completo com TODAS as informações dos filtros
            self.logger.info(f"=== CONVERSÃO AVANÇADA COM FILTROS DE KALMAN ===")
            self.logger.info(f"Pixel: ({pixel_x}, {pixel_y}) -> Local: ({relative_target['x']:.2f}, {relative_target['y']:.2f})")
            self.logger.info(f"Profundidade: {depth_estimate:.2f}±{depth_uncertainty:.2f}m, velocidade: {depth_velocity:.2f}m/s")
            self.logger.info(f"Posição estimada: ({position_estimate[0]:.2f}, {position_estimate[1]:.2f})m")
            self.logger.info(f"Velocidade posição: vx={velocity_estimate[0]:.2f}, vy={velocity_estimate[1]:.2f} m/s")
            self.logger.info(f"Velocidade template: vx={template_velocity[0]:.2f}, vy={template_velocity[1]:.2f} px/s")
            self.logger.info(f"Taxa mudança área: {area_change_rate:.2f} px²/s")
            
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.logger.error(f"Erro na conversão avançada: {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Conversão concluída com sucesso")
import py_trees
import numpy as np
import cv2
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint


class calibracao_dinamica(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.commander = commander
        self.node = None
        
        # Estados da calibração
        self.calibration_state = "INIT"  # INIT -> FIND_FEATURE -> MOVE -> CALCULATE -> DONE
        self.calibration_data = []
        self.conversion_matrix = None
        self.calibration_altitude = None
        
        # Feature tracking com ORB
        self.reference_feature = None
        self.reference_position = None
        self.reference_descriptors = None
        self.reference_keypoints = None
        self.reference_image = None
        self.orb = None
        self.bf_matcher = None
        self.tracking_initialized = False
        
        # Movimentos de calibração (em metros NED)
        self.calibration_moves = [
            {'north': 1.0, 'east': 0.0, 'name': 'Norte 1m'},
            {'north': 0.0, 'east': 1.0, 'name': 'Leste 1m'},
            {'north': -1.0, 'east': 0.0, 'name': 'Sul 1m'},
            {'north': 0.0, 'east': -1.0, 'name': 'Oeste 1m'},
            {'north': 0.0, 'east': 0.0, 'name': 'Retorno ao centro'}
        ]
        self.current_move_index = 0
        self.move_start_time = None
        self.move_timeout = 10.0  # 10 segundos para cada movimento
        self.stabilization_time = 3.0  # Tempo para estabilizar após movimento
        
        # Posição inicial
        self.initial_position = None
        self.current_position = None
        
    def setup(self, **kwargs):
        """Configuração inicial do nodo"""
        try:
            self.node = self.commander
            
            # Configuração do QoS para PX4
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            
            # Subscriber para posição atual
            self.node.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position',
                self.position_callback,
                qos_profile
            )
            
            # Publishers para controle
            self.offboard_control_mode_publisher = self.node.create_publisher(
                OffboardControlMode, 
                '/fmu/in/offboard_control_mode', 
                qos_profile
            )
            
            self.trajectory_setpoint_publisher = self.node.create_publisher(
                TrajectorySetpoint, 
                '/fmu/in/trajectory_setpoint', 
                qos_profile
            )
            
            # Inicializa detector ORB e matcher
            self.orb = cv2.ORB_create(nfeatures=500)
            self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            
            self.logger.info("Calibração dinâmica configurada com sucesso")
            return True
            
        except Exception as e:
            self.logger.error(f"Erro na configuração da calibração: {e}")
            return False
    
    def position_callback(self, msg):
        """Callback para atualizar posição atual"""
        self.current_position = {
            'x': msg.x,  # North
            'y': msg.y,  # East  
            'z': msg.z   # Down (negativo = altitude)
        }
        
        if self.initial_position is None:
            self.initial_position = self.current_position.copy()
            self.calibration_altitude = abs(msg.z)
    
    def initialise(self):
        self.logger.info("Iniciando calibração dinâmica de conversão pixel->NED")
        self.calibration_state = "INIT"
        
    def update(self):
        """Estado principal da calibração"""
        try:
            blackboard = py_trees.blackboard.Blackboard()
            
            if self.calibration_state == "INIT":
                return self.state_init(blackboard)
            elif self.calibration_state == "FIND_FEATURE":
                return self.state_find_feature(blackboard)
            elif self.calibration_state == "MOVE":
                return self.state_move(blackboard)
            elif self.calibration_state == "CALCULATE":
                return self.state_calculate(blackboard)
            elif self.calibration_state == "DONE":
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"Estado desconhecido: {self.calibration_state}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"Erro na calibração dinâmica: {e}")
            return py_trees.common.Status.FAILURE
    
    def state_init(self, blackboard):
        """Estado inicial - aguarda posição e imagem"""
        if self.current_position is None:
            self.logger.info("Aguardando posição do drone...")
            return py_trees.common.Status.RUNNING
            
        if not blackboard.exists("current_image"):
            self.logger.info("Aguardando imagem da câmera...")
            return py_trees.common.Status.RUNNING
        
        self.logger.info(f"Altitude de calibração: {self.calibration_altitude:.2f}m")
        self.calibration_state = "FIND_FEATURE"
        return py_trees.common.Status.RUNNING
    
    def state_find_feature(self, blackboard):
        """Encontra feature próximo ao centro usando ORB"""
        try:
            image = blackboard.get("current_image")
            if image is None:
                return py_trees.common.Status.RUNNING
            
            # Converte para escala de cinza
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            
            # Detecta features ORB na região central
            h, w = gray.shape
            center_x, center_y = w // 2, h // 2
            roi_size = 200  # Região de interesse ao redor do centro
            
            # Define ROI central
            x1 = max(0, center_x - roi_size // 2)
            y1 = max(0, center_y - roi_size // 2)
            x2 = min(w, center_x + roi_size // 2)
            y2 = min(h, center_y + roi_size // 2)
            
            roi = gray[y1:y2, x1:x2]
            
            # Detecta keypoints e descriptors com ORB na ROI
            keypoints, descriptors = self.orb.detectAndCompute(roi, None)
            
            if len(keypoints) == 0 or descriptors is None:
                self.logger.warning("Nenhum feature ORB detectado na região central, tentando novamente...")
                return py_trees.common.Status.RUNNING
            
            # Seleciona o feature mais próximo do centro da ROI
            roi_center = (roi_size // 2, roi_size // 2)
            best_kp_idx = 0
            min_distance = float('inf')
            
            for i, kp in enumerate(keypoints):
                dist = math.sqrt((kp.pt[0] - roi_center[0])**2 + (kp.pt[1] - roi_center[1])**2)
                if dist < min_distance:
                    min_distance = dist
                    best_kp_idx = i
            
            best_kp = keypoints[best_kp_idx]
            
            # Converte coordenadas da ROI para coordenadas da imagem completa
            feature_x = int(best_kp.pt[0] + x1)
            feature_y = int(best_kp.pt[1] + y1)
            self.reference_feature = (feature_x, feature_y)
            
            # Salva a imagem e descriptors de referência
            self.reference_image = gray.copy()
            self.reference_keypoints = keypoints
            self.reference_descriptors = descriptors
            
            self.tracking_initialized = True
            self.reference_position = self.current_position.copy() if self.current_position else {'x': 0, 'y': 0, 'z': 0}
            
            # Salva dados do primeiro ponto (referência)
            self.calibration_data.append({
                'pixel_position': self.reference_feature,
                'ned_position': self.reference_position,
                'move_name': 'Referência'
            })
            
            self.logger.info(f"Feature ORB selecionado em pixel: {self.reference_feature}")
            self.logger.info(f"Posição de referência NED: N={self.reference_position['x']:.2f}, E={self.reference_position['y']:.2f}")
            
            self.calibration_state = "MOVE"
            self.current_move_index = 0
            return py_trees.common.Status.RUNNING
            
        except Exception as e:
            self.logger.error(f"Erro ao encontrar feature ORB: {e}")
            return py_trees.common.Status.FAILURE
    
    def state_move(self, blackboard):
        """Executa movimentos de calibração e rastreia o feature"""
        try:
            if self.current_move_index >= len(self.calibration_moves):
                self.calibration_state = "CALCULATE"
                return py_trees.common.Status.RUNNING
            
            current_move = self.calibration_moves[self.current_move_index]
            
            # Inicia movimento se ainda não iniciou
            if self.move_start_time is None:
                self.move_start_time = time.time()
                target_position = {
                    'x': self.initial_position['x'] + current_move['north'],
                    'y': self.initial_position['y'] + current_move['east'],
                    'z': self.initial_position['z']  # Mantém altitude
                }
                
                self.send_position_command(target_position)
                self.logger.info(f"Iniciando movimento: {current_move['name']}")
                return py_trees.common.Status.RUNNING
            
            # Verifica timeout do movimento
            elapsed_time = time.time() - self.move_start_time
            if elapsed_time > self.move_timeout:
                self.logger.warning(f"Timeout no movimento {current_move['name']}")
                self.move_start_time = None
                self.current_move_index += 1
                return py_trees.common.Status.RUNNING
            
            # Verifica se chegou na posição (tolerância de 20cm)
            target_x = self.initial_position['x'] + current_move['north']
            target_y = self.initial_position['y'] + current_move['east']
            
            if self.current_position is not None:
                distance_error = math.sqrt(
                    (self.current_position['x'] - target_x)**2 + 
                    (self.current_position['y'] - target_y)**2
                )
                
                if distance_error < 0.2 and elapsed_time > self.stabilization_time:
                    # Chegou na posição e estabilizou
                    image = blackboard.get("current_image")
                    if image is not None:
                        # Encontra feature usando matching ORB
                        success, feature_pos = self.match_feature_orb(image)
                        
                        if success:
                            feature_x, feature_y = feature_pos
                            
                            # Salva dados de calibração
                            self.calibration_data.append({
                                'pixel_position': (feature_x, feature_y),
                                'ned_position': self.current_position.copy() if self.current_position else {'x': 0, 'y': 0, 'z': 0},
                                'move_name': current_move['name']
                            })
                            
                            self.logger.info(f"Movimento {current_move['name']} concluído")
                            self.logger.info(f"Feature encontrado em: ({feature_x}, {feature_y})")
                            
                            # Próximo movimento
                            self.move_start_time = None
                            self.current_move_index += 1
                        else:
                            self.logger.error("Perda do tracking do feature")
                            return py_trees.common.Status.FAILURE
            
            return py_trees.common.Status.RUNNING
            
        except Exception as e:
            self.logger.error(f"Erro durante movimento: {e}")
            return py_trees.common.Status.FAILURE
    
    def state_calculate(self, blackboard):
        """Calcula matriz de conversão a partir dos dados coletados"""
        try:
            if len(self.calibration_data) < 2:
                self.logger.error("Dados insuficientes para calibração")
                return py_trees.common.Status.FAILURE
            
            # Prepara dados para regressão linear
            pixel_deltas = []
            ned_deltas = []
            
            reference_pixel = self.calibration_data[0]['pixel_position']
            reference_ned = self.calibration_data[0]['ned_position']
            
            for i in range(1, len(self.calibration_data)):
                data = self.calibration_data[i]
                
                # Calcula deltas
                pixel_delta = (
                    data['pixel_position'][0] - reference_pixel[0],
                    data['pixel_position'][1] - reference_pixel[1]
                )
                
                ned_delta = (
                    data['ned_position']['x'] - reference_ned['x'],
                    data['ned_position']['y'] - reference_ned['y']
                )
                
                pixel_deltas.append(pixel_delta)
                ned_deltas.append(ned_delta)
                
                self.logger.info(f"{data['move_name']}: Pixel Δ{pixel_delta}, NED Δ{ned_delta}")
            
            # Resolve sistema linear: [Δpx, Δpy] = M × [ΔN, ΔE]
            # Montando as equações para mínimos quadrados
            A = []  # Matriz dos movimentos NED
            b_x = []  # Mudanças em pixel X
            b_y = []  # Mudanças em pixel Y
            
            for i, (pixel_delta, ned_delta) in enumerate(zip(pixel_deltas, ned_deltas)):
                A.append([ned_delta[0], ned_delta[1]])  # [ΔN, ΔE]
                b_x.append(pixel_delta[0])  # Δpx
                b_y.append(pixel_delta[1])  # Δpy
            
            A = np.array(A)
            b_x = np.array(b_x)
            b_y = np.array(b_y)
            
            # Resolve para cada direção pixel
            if A.shape[0] >= 2:  # Precisa de pelo menos 2 pontos
                # Regressão linear para pixel X
                coeffs_x = np.linalg.lstsq(A, b_x, rcond=None)[0]
                # Regressão linear para pixel Y  
                coeffs_y = np.linalg.lstsq(A, b_y, rcond=None)[0]
                
                # Monta matriz de conversão 2x2
                self.conversion_matrix = np.array([
                    [coeffs_x[0], coeffs_x[1]],  # [px/m_north, px/m_east]
                    [coeffs_y[0], coeffs_y[1]]   # [py/m_north, py/m_east]
                ])
                
                self.logger.info("Matriz de conversão calculada:")
                self.logger.info(f"  px = {coeffs_x[0]:.2f}*N + {coeffs_x[1]:.2f}*E")
                self.logger.info(f"  py = {coeffs_y[0]:.2f}*N + {coeffs_y[1]:.2f}*E")
                
                # Salva no blackboard para uso pelo img2local
                blackboard.set("calibration_matrix", self.conversion_matrix)
                blackboard.set("calibration_altitude", self.calibration_altitude)
                blackboard.set("calibration_reference_pixel", reference_pixel)
                blackboard.set("calibration_reference_ned", reference_ned)
                blackboard.set("calibration_available", True)
                
                self.logger.info("Calibração dinâmica concluída com sucesso!")
                self.calibration_state = "DONE"
                return py_trees.common.Status.RUNNING
            else:
                self.logger.error("Dados insuficientes para regressão linear")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"Erro no cálculo da matriz de conversão: {e}")
            return py_trees.common.Status.FAILURE
    
    def send_position_command(self, target_position):
        """Envia comando de posição para o drone"""
        try:
            # Publica modo de controle offboard
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            self.offboard_control_mode_publisher.publish(offboard_msg)
            
            # Publica setpoint de trajetória
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
            trajectory_msg.position = [
                float(target_position['x']),
                float(target_position['y']),
                float(target_position['z'])
            ]
            trajectory_msg.yaw = 0.0  # Mantém orientação
            self.trajectory_setpoint_publisher.publish(trajectory_msg)
            
        except Exception as e:
            self.logger.error(f"Erro ao enviar comando de posição: {e}")
    
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Calibração dinâmica concluída com sucesso")
        else:
            self.logger.info("Calibração dinâmica finalizada")
    
    def match_feature_orb(self, current_image):
        """
        Encontra o feature de referência na imagem atual usando matching ORB
        """
        try:
            if self.reference_descriptors is None:
                return False, None
            
            # Converte para escala de cinza
            gray = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY) if len(current_image.shape) == 3 else current_image
            
            # Detecta features na imagem atual
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)
            
            if descriptors is None or len(keypoints) == 0:
                self.logger.warning("Nenhum feature ORB detectado na imagem atual")
                return False, None
            
            # Faz matching entre descriptors de referência e atuais
            matches = self.bf_matcher.match(self.reference_descriptors, descriptors)
            
            if len(matches) == 0:
                self.logger.warning("Nenhum match encontrado")
                return False, None
            
            # Ordena matches por distância (melhor primeiro)
            matches = sorted(matches, key=lambda x: x.distance)
            
            # Pega o melhor match
            best_match = matches[0]
            
            # Verifica se o match é bom o suficiente
            if best_match.distance > 50:  # Threshold de qualidade
                self.logger.warning(f"Match de baixa qualidade: {best_match.distance}")
                return False, None
            
            # Obtém posição do keypoint correspondente na imagem atual
            matched_kp = keypoints[best_match.trainIdx]
            feature_x = int(matched_kp.pt[0])
            feature_y = int(matched_kp.pt[1])
            
            self.logger.info(f"Feature ORB encontrado com distância: {best_match.distance:.2f}")
            
            return True, (feature_x, feature_y)
            
        except Exception as e:
            self.logger.error(f"Erro no matching ORB: {e}")
            return False, None

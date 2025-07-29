import py_trees
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition


class img2local(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        
        # Parâmetros da câmera (ajustar conforme sua câmera)
        # Câmera aponta para a frente do drone
        self.camera_fov_horizontal = math.radians(60)  # FOV horizontal em radianos
        self.camera_fov_vertical = math.radians(45)    # FOV vertical em radianos
        self.image_width = 1280   # Largura da imagem em pixels
        self.image_height = 960   # Altura da imagem em pixels
        
        # NOVO: Parâmetros de distância focal para maior precisão
        # Opção 1: Distância focal em mm (se conhecida)
        self.focal_length_mm = 3.6  # Exemplo: 3.6mm (ajustar conforme sua câmera)
        self.sensor_width_mm = 6.17  # Largura do sensor em mm (ajustar conforme sua câmera)
        self.sensor_height_mm = 4.63  # Altura do sensor em mm (ajustar conforme sua câmera)
        
        # Opção 2: Calcular distância focal em pixels a partir do FOV
        self.use_focal_length = True  # True = usar focal length, False = usar FOV
        
        # Calcula distância focal em pixels
        if self.use_focal_length and self.focal_length_mm > 0:
            # Converte focal length de mm para pixels
            self.focal_length_x_pixels = (self.focal_length_mm * self.image_width) / self.sensor_width_mm
            self.focal_length_y_pixels = (self.focal_length_mm * self.image_height) / self.sensor_height_mm
        else:
            # Calcula focal length em pixels a partir do FOV
            self.focal_length_x_pixels = (self.image_width / 2) / math.tan(self.camera_fov_horizontal / 2)
            self.focal_length_y_pixels = (self.image_height / 2) / math.tan(self.camera_fov_vertical / 2)
        
        # Inicializa variáveis
        self.current_altitude = None
        self.node = None
        
    def setup(self, **kwargs):
        # Verifica se o parâmetro 'node' está disponível
        if 'node' not in kwargs:
            self.logger.warning("Parâmetro 'node' não fornecido - subscriber não será criado")
            return True
            
        self.node = kwargs['node']
        # Initialize ROS2 components com o mesmo QoS do PX4Commander
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Adiciona o subscriber para a altitude atual
        self.node.create_subscription(
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
        self.logger.info("Iniciando conversão de coordenadas pixel para NED")
        
    def update(self):
        """
        Converte a posição em pixels para coordenadas NED relativas ao drone
        """
        try:
            blackboard = py_trees.blackboard.Blackboard()
            
            # Obtém a posição em pixels do local seguro
            if not blackboard.exists("local_seguro_pixel"):
                self.logger.error("Posição do local seguro não encontrada no blackboard")
                return py_trees.common.Status.FAILURE
                
            pixel_pos = blackboard.get("local_seguro_pixel")
            pixel_x, pixel_y = pixel_pos
            
            # Adiciona validação das coordenadas pixel
            self.logger.info(f"Coordenadas pixel recebidas: ({pixel_x}, {pixel_y})")
            self.logger.info(f"Dimensões da imagem configuradas: {self.image_width}x{self.image_height}")
            
            # Clamps as coordenadas dentro dos limites da imagem
            # Para Y: garante que seja >= 480 (metade inferior da imagem)
            pixel_x = max(0, min(pixel_x, self.image_width - 1))
            pixel_y = max(480, min(pixel_y, self.image_height - 1))  # Y mínimo 480
            
            if pixel_x != pixel_pos[0] or pixel_y != pixel_pos[1]:
                self.logger.warning(f"Coordenadas pixel ajustadas para: ({pixel_x}, {pixel_y})")
            
            # Obtém a altitude atual do drone
            if self.current_altitude is None:
                self.logger.warning("Posição atual do drone não disponível")
                return py_trees.common.Status.RUNNING
                
            altitude = abs(self.current_altitude)  # Altitude absoluta (PX4 usa Z negativo para cima)
            
            # Adiciona validação de altitude mínima
            if altitude < 0.1:  # Muito baixo, pode ser erro de leitura
                self.logger.warning(f"Altitude muito baixa ou zero: {altitude:.2f}m - usando valor mínimo")
                altitude = 1.0  # Assume altitude mínima de 1m para cálculos
            
            self.logger.info(f"Altitude atual do drone: {self.current_altitude:.2f}m, Altitude calculada: {altitude:.2f}m") 
            
            # Converte pixels para ângulos relativos ao centro da imagem
            # Centro da imagem como referência (0,0)
            center_x = self.image_width / 2
            center_y = self.image_height / 2
            
            # Offset em pixels do centro
            offset_x_pixels = pixel_x - center_x
            offset_y_pixels = pixel_y - center_y
            
            # MÉTODO MELHORADO: Converte para ângulos usando distância focal
            if self.use_focal_length:
                # Método mais preciso usando distância focal
                angle_x = math.atan(offset_x_pixels / self.focal_length_x_pixels)
                angle_y = math.atan(offset_y_pixels / self.focal_length_y_pixels)
                
                self.logger.info(f"Usando distância focal - Focal X: {self.focal_length_x_pixels:.1f}px, Focal Y: {self.focal_length_y_pixels:.1f}px")
                self.logger.info(f"Ângulos calculados (focal): X={math.degrees(angle_x):.2f}°, Y={math.degrees(angle_y):.2f}°")
            else:
                # Método original usando FOV (linear)
                angle_x = (offset_x_pixels / self.image_width) * self.camera_fov_horizontal
                angle_y = (offset_y_pixels / self.image_height) * self.camera_fov_vertical
                
                self.logger.info(f"Usando FOV linear - Ângulos calculados: X={math.degrees(angle_x):.2f}°, Y={math.degrees(angle_y):.2f}°")
            
            # NOVA FUNCIONALIDADE: Verifica se há calibração dinâmica disponível
            if blackboard.exists("calibration_available") and blackboard.get("calibration_available"):
                # Usa calibração dinâmica (método mais preciso)
                distance_north, distance_east = self.convert_with_calibration(
                    offset_x_pixels, offset_y_pixels, altitude, blackboard
                )
                self.logger.info("Usando conversão com calibração dinâmica")
            else:
                # Fallback: conversão trigonométrica tradicional
                distance_north = altitude * math.tan(angle_y)  # Positivo = à frente
                distance_east = -altitude * math.tan(angle_x)   # Negativo = à esquerda (oeste), Positivo = à direita (leste)
                self.logger.info("Usando conversão trigonométrica (sem calibração)")
            
            # Em NED: North (X), East (Y), Down (Z)
            ned_target = {
                'x': distance_north,   # Norte
                'y': distance_east,    # Leste  
                'z': 0                 # Manter mesma altitude (diferença Z = 0)
            }
            
            # Armazena no blackboard
            blackboard.set("target_ned_position", ned_target)
            
            self.logger.info(f"Conversão pixel->NED: pixel({pixel_x}, {pixel_y}) -> NED({distance_north:.2f}, {distance_east:.2f}, 0)")
            self.logger.info(f"Altitude do drone: {altitude:.2f}m")
            
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.logger.error(f"Erro na conversão pixel para NED: {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Conversão pixel->NED concluída com sucesso")
        else:
            self.logger.info("Conversão pixel->NED finalizada")
    
    def convert_with_calibration(self, offset_x_pixels, offset_y_pixels, altitude, blackboard):
        """
        Converte offset de pixels para coordenadas NED usando calibração dinâmica
        """
        try:
            # Obtém dados de calibração
            calibration_matrix = blackboard.get("calibration_matrix")
            calibration_altitude = blackboard.get("calibration_altitude")
            
            if calibration_matrix is None or calibration_altitude is None:
                self.logger.warning("Dados de calibração incompletos, usando método trigonométrico")
                # Fallback para método trigonométrico
                angle_x = math.atan(offset_x_pixels / self.focal_length_x_pixels) if self.use_focal_length else (offset_x_pixels / self.image_width) * self.camera_fov_horizontal
                angle_y = math.atan(offset_y_pixels / self.focal_length_y_pixels) if self.use_focal_length else (offset_y_pixels / self.image_height) * self.camera_fov_vertical
                
                distance_north = altitude * math.tan(angle_y)
                distance_east = -altitude * math.tan(angle_x)
                return distance_north, distance_east
            
            # Converte matriz para numpy se necessário
            if not isinstance(calibration_matrix, np.ndarray):
                calibration_matrix = np.array(calibration_matrix)
            
            # Inverte a matriz de conversão: [Δpx, Δpy] = M × [ΔN, ΔE]
            # Queremos: [ΔN, ΔE] = M⁻¹ × [Δpx, Δpy]
            try:
                inv_matrix = np.linalg.inv(calibration_matrix)
            except np.linalg.LinAlgError:
                self.logger.error("Matriz de calibração não é inversível")
                # Fallback para método trigonométrico
                angle_x = math.atan(offset_x_pixels / self.focal_length_x_pixels) if self.use_focal_length else (offset_x_pixels / self.image_width) * self.camera_fov_horizontal
                angle_y = math.atan(offset_y_pixels / self.focal_length_y_pixels) if self.use_focal_length else (offset_y_pixels / self.image_height) * self.camera_fov_vertical
                
                distance_north = altitude * math.tan(angle_y)
                distance_east = -altitude * math.tan(angle_x)
                return distance_north, distance_east
            
            # Converte offset de pixels para mudança NED por pixel
            pixel_delta = np.array([offset_x_pixels, offset_y_pixels])
            ned_delta_per_pixel = inv_matrix @ pixel_delta
            
            # Aplica correção de altitude (escala com a altura)
            # A calibração foi feita em uma altitude específica
            altitude_scale = altitude / calibration_altitude
            
            # Calcula distâncias finais NED
            distance_north = ned_delta_per_pixel[0] * altitude_scale
            distance_east = ned_delta_per_pixel[1] * altitude_scale
            
            self.logger.info(f"Calibração aplicada - Escala altitude: {altitude_scale:.2f}")
            self.logger.info(f"Delta NED bruto: N={ned_delta_per_pixel[0]:.3f}, E={ned_delta_per_pixel[1]:.3f}")
            
            return distance_north, distance_east
            
        except Exception as e:
            self.logger.error(f"Erro na conversão com calibração: {e}")
            # Fallback para método trigonométrico
            angle_x = math.atan(offset_x_pixels / self.focal_length_x_pixels) if self.use_focal_length else (offset_x_pixels / self.image_width) * self.camera_fov_horizontal
            angle_y = math.atan(offset_y_pixels / self.focal_length_y_pixels) if self.use_focal_length else (offset_y_pixels / self.image_height) * self.camera_fov_vertical
            
            distance_north = altitude * math.tan(angle_y)
            distance_east = -altitude * math.tan(angle_x)
            return distance_north, distance_east

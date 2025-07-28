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
            
            # Converte para ângulos
            angle_x = (offset_x_pixels / self.image_width) * self.camera_fov_horizontal
            angle_y = (offset_y_pixels / self.image_height) * self.camera_fov_vertical
            
            # Converte ângulos para distâncias no solo usando trigonometria
            # Câmera aponta para a frente - usa altitude para calcular distância no solo
            # Para câmera frontal: Y da imagem corresponde à distância à frente (X em NED)
            # X da imagem corresponde ao lado esquerdo/direito (Y em NED)
            
            # Distância à frente (North em NED) - baseada no ângulo vertical da câmera
            distance_north = altitude * math.tan(angle_y)  # Positivo = à frente
            
            # Distância lateral (East em NED) - baseada no ângulo horizontal da câmera  
            distance_east = altitude * math.tan(angle_x)   # Positivo = à direita
            
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

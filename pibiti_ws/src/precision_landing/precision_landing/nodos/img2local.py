import py_trees
import numpy as np
import math


class img2local(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.commander = commander
        
        # Parâmetros da câmera (ajustar conforme sua câmera)
        # Exemplo para câmera com FOV de 60 graus
        self.camera_fov_horizontal = math.radians(60)  # FOV horizontal em radianos
        self.camera_fov_vertical = math.radians(45)    # FOV vertical em radianos
        self.image_width = 640   # Largura da imagem em pixels
        self.image_height = 480  # Altura da imagem em pixels
        
    def setup(self):
        self.logger.debug("Configurando conversão imagem para coordenadas locais")
        
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
                self.logger.warn("Posição do local seguro não encontrada no blackboard")
                return py_trees.common.Status.FAILURE
                
            pixel_pos = blackboard.get("local_seguro_pixel")
            pixel_x, pixel_y = pixel_pos
            
            # Obtém a altitude atual do drone
            if not hasattr(self.commander, 'curr_position') or self.commander.curr_position is None:
                self.logger.warn("Posição atual do drone não disponível")
                return py_trees.common.Status.RUNNING
                
            current_pos = self.commander.curr_position
            altitude = -current_pos[2]  # Altitude positiva (NED usa Z negativo para cima)
            
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
            # Assumindo que a câmera aponta para baixo
            distance_x = altitude * math.tan(angle_x)  # Norte (positivo = norte)
            distance_y = altitude * math.tan(angle_y)  # Leste (positivo = leste)
            
            # Em NED: North (X), East (Y), Down (Z)
            ned_target = {
                'x': distance_x,   # Norte
                'y': distance_y,   # Leste  
                'z': 0             # Manter mesma altitude (diferença Z = 0)
            }
            
            # Armazena no blackboard
            blackboard.set("target_ned_position", ned_target)
            
            self.logger.info(f"Conversão pixel->NED: pixel({pixel_x}, {pixel_y}) -> NED({distance_x:.2f}, {distance_y:.2f}, 0)")
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

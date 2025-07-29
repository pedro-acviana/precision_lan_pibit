import py_trees
import numpy as np
import math
import pickle
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition


class img2local(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        
        # Parâmetros intrínsecos da câmera
        self.fx = 1009.622747206156419  # focal length x (pixels)
        self.fy = 1007.783677020086202  # focal length y (pixels)
        self.cx = 294.9315450028845476  # principal point x
        self.cy = 233.5219972126571619  # principal point y
        
        # Dimensões da imagem
        self.image_width = 640   # Largura da imagem em pixels
        self.image_height = 480   # Altura da imagem em pixels
        
        # Matriz de calibração intrínseca K
        self.K = np.array([
            [self.fx, 0,       self.cx],
            [0,       self.fy, self.cy],
            [0,       0,       1]
        ])
        
        # Matriz de rotação (assumindo câmera apontando para a frente)
        # Para câmera apontando para frente: sem rotação (matriz identidade)
        self.R = np.eye(3)
        
        # Vetor de translação (sem translação adicional)
        self.t = np.array([0, 0, 0])
        
        # Matriz de projeção P = K [R|t]
        self.P = np.dot(self.K, np.hstack((self.R, self.t.reshape(3,1))))
        
        # Inicializa variáveis
        self.current_altitude = None
        self.node = None
        
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

        self.node.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )

        return True
    
    def local_position_callback(self, msg):
        """Callback para atualizar a altitude atual"""
        self.current_altitude = -msg.z  # Inverte Z para ter altitude positiva
        
    def initialise(self):
        self.logger.info("Iniciando conversão de coordenadas pixel para coordenadas relativas")
    
    def pixel_to_relative_position(self, u, v, altitude):
        """Converte coordenadas de pixel para posição relativa ao drone usando matriz de projeção"""
        try:
            # Coordenadas homogêneas do pixel
            pixel_hom = np.array([u, v, 1])
            
            # Remove a distorção (normaliza as coordenadas) usando K inversa
            normalized = np.linalg.inv(self.K) @ pixel_hom
            
            # Calcula a direção no espaço 3D (assumindo Z = 1)
            direction = normalized / normalized[2] if normalized[2] != 0 else normalized
            
            # Escala pela altitude real (Z = altura do drone)
            point_3d = direction * altitude
            
            # Ajusta o sistema de coordenadas para câmera apontando para frente:
            # - Nosso X (frente) é o -Y da câmera (invertido porque Y da imagem cresce para baixo)
            # - Nosso Y (direita) é o X da câmera  
            # - Z (para baixo) é o Z da câmera
            relative_position = np.array([
                -point_3d[1],  # X = frente (negativo do Y da câmera)
                point_3d[0],   # Y = direita (X da câmera)
                0              # Z = mantém altitude
            ])
            
            return relative_position
            
        except Exception as e:
            self.logger.error(f"Erro no cálculo de posição relativa: {e}")
            return np.array([0, 0, 0])
    
    def update(self):
        try:
            blackboard = py_trees.blackboard.Blackboard()
            
            if not blackboard.exists("local_seguro_pixel"):
                self.logger.error("Posição do local seguro não encontrada no blackboard")
                return py_trees.common.Status.FAILURE
                
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
            
            # Usa o método de conversão baseado em matriz de projeção
            relative_pos = self.pixel_to_relative_position(pixel_x, pixel_y, altitude)
            
            relative_target = {
                'x': relative_pos[0],  # Norte/Sul (frente/trás)
                'y': relative_pos[1],  # Leste/Oeste (esquerda/direita)
                'z': 0                 # Altitude
            }
            
            blackboard.set("target_relative_position", relative_target)
            
            self.logger.info(f"Conversão: pixel({pixel_x}, {pixel_y}) -> local({relative_target['x']:.2f}, {relative_target['y']:.2f}, 0)")
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.logger.error(f"Erro na conversão: {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Conversão concluída com sucesso")
"""
Filtro de Kalman para fusão de dados visuais e estimativa de profundidade
Baseado nos conceitos do Capítulo 3 e Apêndice H do livro de Corke
"""

import numpy as np
import time

# Import condicional do OpenCV
try:
    import cv2
    opencv_available = True
except ImportError:
    opencv_available = False


class DepthEstimationKalmanFilter:
    """
    Filtro de Kalman para estimar a profundidade do alvo usando:
    - Variação do tamanho aparente do alvo
    - Dados de posição do drone (IMU)
    - Medições visuais da câmera
    """
    
    def __init__(self, initial_depth=5.0, process_noise=0.1, measurement_noise=0.5):
        """
        Inicializa o filtro de Kalman para estimativa de profundidade
        
        Args:
            initial_depth: Estimativa inicial da profundidade (metros)
            process_noise: Ruído do processo (variação da profundidade)
            measurement_noise: Ruído das medições visuais
        """
        # Estado: [profundidade, velocidade_profundidade]
        self.state = np.array([initial_depth, 0.0])
        
        # Matriz de covariância do estado
        self.P = np.array([[1.0, 0.0],
                          [0.0, 1.0]])
        
        # Matriz de transição de estado (modelo de movimento constante)
        self.F = np.array([[1.0, 0.0],
                          [0.0, 1.0]])
        
        # Matriz de observação (observamos apenas a profundidade)
        self.H = np.array([[1.0, 0.0]])
        
        # Ruído do processo
        self.Q = np.array([[process_noise, 0.0],
                          [0.0, process_noise * 0.1]])
        
        # Ruído da medição
        self.R = np.array([[measurement_noise]])
        
        self.last_update_time = time.time()
        
    def predict(self, dt=None):
        """
        Passo de predição do filtro de Kalman
        """
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
        
        # Atualiza matriz de transição com delta de tempo
        self.F[0, 1] = dt
        
        # Predição do estado
        self.state = self.F @ self.state
        
        # Predição da covariância
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, depth_measurement):
        """
        Passo de atualização do filtro de Kalman
        
        Args:
            depth_measurement: Medição da profundidade (metros)
        """
        # Inovação (diferença entre medição e predição)
        y = depth_measurement - self.H @ self.state
        
        # Covariância da inovação
        S = self.H @ self.P @ self.H.T + self.R
        
        # Ganho de Kalman
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Atualização do estado
        self.state = self.state + K @ y
        
        # Atualização da covariância
        I = np.eye(len(self.state))
        self.P = (I - K @ self.H) @ self.P
        
    def get_depth_estimate(self):
        """
        Retorna a estimativa atual da profundidade
        """
        return max(0.1, self.state[0])  # Garante profundidade mínima positiva
    
    def get_depth_uncertainty(self):
        """
        Retorna a incerteza da estimativa de profundidade
        """
        return np.sqrt(self.P[0, 0])


class PositionKalmanFilter:
    """
    Filtro de Kalman para fusão de dados visuais e IMU
    Estima posição e velocidade do drone com maior precisão
    """
    
    def __init__(self, process_noise=0.1, measurement_noise_vision=0.5, measurement_noise_imu=0.1):
        """
        Inicializa o filtro de Kalman para posição
        
        Estado: [x, y, vx, vy] onde:
        - x, y: posição em NED
        - vx, vy: velocidade em NED
        """
        # Estado inicial
        self.state = np.zeros(4)  # [x, y, vx, vy]
        
        # Matriz de covariância do estado
        self.P = np.eye(4) * 1.0
        
        # Matriz de transição de estado
        self.F = np.eye(4)
        
        # Matriz de observação para medições visuais (posição apenas)
        self.H_vision = np.array([[1.0, 0.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0, 0.0]])
        
        # Matriz de observação para IMU (velocidade apenas)
        self.H_imu = np.array([[0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])
        
        # Ruído do processo
        self.Q = np.eye(4) * process_noise
        
        # Ruído das medições
        self.R_vision = np.eye(2) * measurement_noise_vision
        self.R_imu = np.eye(2) * measurement_noise_imu
        
        self.last_update_time = time.time()
        
    def predict(self, dt=None):
        """
        Passo de predição usando modelo de velocidade constante
        """
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
        
        # Atualiza matriz de transição
        self.F[0, 2] = dt  # x = x + vx*dt
        self.F[1, 3] = dt  # y = y + vy*dt
        
        # Predição
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update_vision(self, position_measurement):
        """
        Atualiza com medição visual da posição
        
        Args:
            position_measurement: [x, y] em metros NED
        """
        position_measurement = np.array(position_measurement)
        
        # Inovação
        y = position_measurement - self.H_vision @ self.state
        
        # Covariância da inovação
        S = self.H_vision @ self.P @ self.H_vision.T + self.R_vision
        
        # Ganho de Kalman
        K = self.P @ self.H_vision.T @ np.linalg.inv(S)
        
        # Atualização
        self.state = self.state + K @ y
        I = np.eye(len(self.state))
        self.P = (I - K @ self.H_vision) @ self.P
        
    def update_imu(self, velocity_measurement):
        """
        Atualiza com medição da IMU (velocidade)
        
        Args:
            velocity_measurement: [vx, vy] em m/s NED
        """
        velocity_measurement = np.array(velocity_measurement)
        
        # Inovação
        y = velocity_measurement - self.H_imu @ self.state
        
        # Covariância da inovação
        S = self.H_imu @ self.P @ self.H_imu.T + self.R_imu
        
        # Ganho de Kalman
        K = self.P @ self.H_imu.T @ np.linalg.inv(S)
        
        # Atualização
        self.state = self.state + K @ y
        I = np.eye(len(self.state))
        self.P = (I - K @ self.H_imu) @ self.P
        
    def get_position_estimate(self):
        """
        Retorna estimativa da posição [x, y]
        """
        return self.state[:2]
        
    def get_velocity_estimate(self):
        """
        Retorna estimativa da velocidade [vx, vy]
        """
        return self.state[2:]
        
    def get_position_uncertainty(self):
        """
        Retorna incerteza da posição
        """
        return np.sqrt(np.diag(self.P[:2, :2]))
        
    def get_velocity_uncertainty(self):
        """
        Retorna incerteza da velocidade
        """
        return np.sqrt(np.diag(self.P[2:, 2:]))


class HomographyTracker:
    """
    Rastreamento usando homografia planar (versão simplificada)
    Baseado no Capítulo 14 do livro de Corke
    """
    
    def __init__(self, reference_features=None):
        """
        Inicializa o rastreador de homografia
        
        Args:
            reference_features: Features de referência do frame inicial
        """
        self.reference_features = reference_features
        self.reference_descriptors = None
        self.opencv_available = opencv_available
        
        # Só inicializa se OpenCV estiver disponível
        if self.opencv_available:
            self.detector = None
            self.matcher = None
            # Não inicializa detectores aqui para evitar erros de importação
        
    def set_reference(self, image, target_region=None):
        """
        Define a imagem de referência e extrai features
        Versão simplificada que sempre retorna False se OpenCV não estiver disponível
        """
        if not self.opencv_available:
            return False
        # Por simplicidade, não implementa detecção real
        return False
        
    def track(self, current_image):
        """
        Rastreia features na imagem atual e calcula homografia
        Versão simplificada que sempre retorna None
        """
        if not self.opencv_available:
            return None
        # Por simplicidade, não implementa tracking real
        return None

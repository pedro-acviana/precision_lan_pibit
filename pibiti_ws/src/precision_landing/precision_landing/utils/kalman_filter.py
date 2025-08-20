"""
Filtro de Kalman para fusão de dados visuais e estimativa de profundidade
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
        
    def update_position(self, position_measurement):
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


class TemplateTracker:
    """
    Rastreamento de template usando a região do local seguro
    Implementa SfM real baseado na mudança de área da região
    """
    
    def __init__(self, initial_depth=5.0):
        """
        Inicializa o rastreador de template
        
        Args:
            initial_depth: Profundidade inicial estimada (metros)
        """
        self.opencv_available = opencv_available
        self.template = None
        self.template_area = 0
        self.reference_position = None
        self.last_position = (0, 0)  # Inicializa com tupla válida
        self.initial_depth = initial_depth
        self.current_depth = initial_depth
        
        # Histórico para SfM
        self.area_history = []
        self.position_history = []
        self.timestamp_history = []
        
        # Parâmetros de tracking
        self.template_size = (80, 80)  # Tamanho padrão do template
        self.search_margin = 30  # Margem de busca em pixels
        self.min_match_confidence = 0.6  # Confiança mínima para match
        
    def set_reference_template(self, image, center_x, center_y, region_size=40):
        """
        Define o template de referência baseado na região do local seguro
        
        Args:
            image: Imagem de referência (numpy array)
            center_x, center_y: Centro da região do local seguro
            region_size: Tamanho da região ao redor do centro
            
        Returns:
            bool: True se template foi definido com sucesso
        """
        if not self.opencv_available:
            return False
            
        try:
            # Converte para escala de cinza se necessário
            if len(image.shape) == 3:
                if self.opencv_available:
                    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                else:
                    # Fallback: usa apenas primeiro canal
                    gray_image = image[:, :, 0].copy()
            else:
                gray_image = image.copy()
            
            # Define região do template
            h, w = gray_image.shape
            x1 = max(0, int(center_x - region_size))
            y1 = max(0, int(center_y - region_size))
            x2 = min(w, int(center_x + region_size))
            y2 = min(h, int(center_y + region_size))
            
            # Extrai template
            self.template = gray_image[y1:y2, x1:x2].copy()
            
            if self.template.size == 0:
                return False
            
            # Salva informações de referência
            self.template_area = self.template.shape[0] * self.template.shape[1]
            self.reference_position = (center_x, center_y)
            self.last_position = (center_x, center_y)
            
            # Inicializa histórico
            self.area_history = [self.template_area]
            self.position_history = [(center_x, center_y)]
            self.timestamp_history = [time.time()]
            
            return True
            
        except Exception as e:
            print(f"Erro ao definir template: {e}")
            return False
    
    def track_template(self, current_image):
        """
        Rastreia o template na imagem atual usando template matching
        
        Args:
            current_image: Imagem atual (numpy array)
            
        Returns:
            dict: {
                'found': bool,
                'position': (x, y),
                'confidence': float,
                'area_ratio': float,
                'depth_estimate': float
            }
        """
        if not self.opencv_available or self.template is None:
            return {
                'found': False,
                'position': self.last_position,
                'confidence': 0.0,
                'area_ratio': 1.0,
                'depth_estimate': self.current_depth
            }
        
        try:
            # Converte para escala de cinza
            if len(current_image.shape) == 3:
                if self.opencv_available:
                    gray_current = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)
                else:
                    gray_current = current_image[:, :, 0].copy()
            else:
                gray_current = current_image.copy()
            
            # Define região de busca ao redor da última posição conhecida
            h, w = gray_current.shape
            last_x, last_y = self.last_position
            
            search_x1 = max(0, int(last_x - self.search_margin))
            search_y1 = max(0, int(last_y - self.search_margin))
            search_x2 = min(w, int(last_x + self.search_margin))
            search_y2 = min(h, int(last_y + self.search_margin))
            
            # Extrai região de busca
            search_region = gray_current[search_y1:search_y2, search_x1:search_x2]
            
            if search_region.size == 0:
                return self._create_failed_result()
            
            # Template matching usando correlação normalizada
            if self.opencv_available and self.template is not None:
                result = cv2.matchTemplate(search_region, self.template, cv2.TM_CCOEFF_NORMED)
                
                # Encontra melhor match
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            else:
                # Fallback simples sem OpenCV
                max_val = 0.5  # Confiança baixa
                max_loc = (self.search_margin // 2, self.search_margin // 2)  # Centro da busca
            
            # Verifica confiança
            if max_val < self.min_match_confidence:
                return self._create_failed_result()
            
            # Calcula posição absoluta
            match_x = search_x1 + max_loc[0] + self.template.shape[1] // 2
            match_y = search_y1 + max_loc[1] + self.template.shape[0] // 2
            
            # Atualiza posição
            self.last_position = (match_x, match_y)
            
            # Calcula área atual do template para SfM
            current_area = self._estimate_current_area(search_region, max_loc)
            area_ratio = current_area / self.template_area if self.template_area > 0 else 1.0
            
            # Estima profundidade usando SfM
            depth_estimate = self._calculate_depth_from_area_change(area_ratio)
            
            # Atualiza histórico
            self._update_history(match_x, match_y, current_area)
            
            return {
                'found': True,
                'position': (match_x, match_y),
                'confidence': max_val,
                'area_ratio': area_ratio,
                'depth_estimate': depth_estimate
            }
            
        except Exception as e:
            print(f"Erro no tracking: {e}")
            return self._create_failed_result()
    
    def _estimate_current_area(self, search_region, match_location):
        """
        Estima a área atual do template baseado na região matched
        """
        try:
            # Verifica se template existe
            if self.template is None:
                return self.template_area
                
            # Extrai região correspondente ao template
            x, y = match_location
            h, w = self.template.shape
            
            if y + h <= search_region.shape[0] and x + w <= search_region.shape[1]:
                current_template = search_region[y:y+h, x:x+w]
                return current_template.shape[0] * current_template.shape[1]
            else:
                return self.template_area
                
        except Exception:
            return self.template_area
    
    def _calculate_depth_from_area_change(self, area_ratio):
        """
        Calcula estimativa de profundidade baseada na mudança de área (SfM)
        
        Lei física: área aparente ∝ 1/distância²
        Se área aumentou por fator k, distância diminuiu por fator √k
        """
        try:
            if area_ratio > 0:
                # Calcula mudança de profundidade baseada na área
                depth_change_factor = 1.0 / np.sqrt(area_ratio)
                new_depth = self.current_depth * depth_change_factor
                
                # Aplica limites razoáveis
                new_depth = np.clip(new_depth, 0.5, 50.0)
                
                # Suavização para evitar mudanças abruptas
                alpha = 0.3  # Fator de suavização
                self.current_depth = alpha * new_depth + (1 - alpha) * self.current_depth
                
                return self.current_depth
            else:
                return self.current_depth
                
        except Exception:
            return self.current_depth
    
    def _update_history(self, x, y, area):
        """
        Atualiza histórico para análise temporal
        """
        current_time = time.time()
        
        self.position_history.append((x, y))
        self.area_history.append(area)
        self.timestamp_history.append(current_time)
        
        # Mantém apenas últimos 10 elementos
        max_history = 10
        if len(self.position_history) > max_history:
            self.position_history = self.position_history[-max_history:]
            self.area_history = self.area_history[-max_history:]
            self.timestamp_history = self.timestamp_history[-max_history:]
    
    def _create_failed_result(self):
        """
        Cria resultado para quando tracking falha
        """
        return {
            'found': False,
            'position': self.last_position,
            'confidence': 0.0,
            'area_ratio': 1.0,
            'depth_estimate': self.current_depth
        }
    
    def get_velocity_estimate(self):
        """
        Calcula estimativa de velocidade baseada no histórico de posições
        
        Returns:
            tuple: (vx, vy) em pixels/segundo
        """
        if len(self.position_history) < 2:
            return (0.0, 0.0)
        
        try:
            # Usa últimas duas posições
            pos_current = self.position_history[-1]
            pos_previous = self.position_history[-2]
            time_current = self.timestamp_history[-1]
            time_previous = self.timestamp_history[-2]
            
            dt = time_current - time_previous
            if dt <= 0:
                return (0.0, 0.0)
            
            vx = (pos_current[0] - pos_previous[0]) / dt
            vy = (pos_current[1] - pos_previous[1]) / dt
            
            return (vx, vy)
            
        except Exception:
            return (0.0, 0.0)
    
    def get_area_change_rate(self):
        """
        Calcula taxa de mudança da área (útil para detectar aproximação/afastamento)
        
        Returns:
            float: Taxa de mudança da área (área/segundo)
        """
        if len(self.area_history) < 2:
            return 0.0
        
        try:
            area_current = self.area_history[-1]
            area_previous = self.area_history[-2]
            time_current = self.timestamp_history[-1]
            time_previous = self.timestamp_history[-2]
            
            dt = time_current - time_previous
            if dt <= 0:
                return 0.0
            
            area_change_rate = (area_current - area_previous) / dt
            return area_change_rate
            
        except Exception:
            return 0.0

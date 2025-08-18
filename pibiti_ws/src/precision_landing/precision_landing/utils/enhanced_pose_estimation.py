"""
Conversão melhorada de coordenadas 2D para 3D usando pose estimation
"""

import numpy as np
import math
from typing import Tuple, Optional, List

# Import condicional do OpenCV
try:
    import cv2
    opencv_available = True
except ImportError:
    opencv_available = False


class EnhancedPose2D3D:
    """
    Classe para conversão precisa de coordenadas 2D para 3D usando:
    - Parâmetros intrínsecos da câmera
    - Estimativa de profundidade relativa
    - Visual servoing para depth estimation
    """
    
    def __init__(self, camera_matrix, dist_coeffs=None, image_size=(1280, 960)):
        """
        Inicializa o conversor de pose
        
        Args:
            camera_matrix: Matriz intrínseca da câmera K (3x3)
            dist_coeffs: Coeficientes de distorção da lente
            image_size: Tamanho da imagem (width, height)
        """
        self.K = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        self.image_width, self.image_height = image_size
        
        # Extrai parâmetros intrínsecos
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]
        
        # Histórico para estimativa de profundidade relativa
        self.feature_history = []
        self.depth_estimates = []
        
    def pixel_to_normalized_coordinates(self, u: float, v: float) -> Tuple[float, float]:
        """
        Converte coordenadas de pixel para coordenadas normalizadas da câmera
        
        Args:
            u, v: Coordenadas de pixel
            
        Returns:
            x_norm, y_norm: Coordenadas normalizadas
        """
        # Corrige distorção se necessário
        if np.any(self.dist_coeffs):
            points = np.array([[[u, v]]], dtype=np.float32)
            undistorted = cv2.undistortPoints(points, self.K, self.dist_coeffs, P=self.K)
            u, v = undistorted[0, 0]
        
        # Normaliza usando parâmetros intrínsecos
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        
        return x_norm, y_norm
    
    def estimate_depth_from_apparent_size(self, current_features: List, reference_features: Optional[List] = None) -> Optional[float]:
        """
        Estima profundidade baseada na variação do tamanho aparente das features
        Implementa conceitos de Structure-from-Motion do Capítulo 14
        
        Args:
            current_features: Features detectadas no frame atual (ou área do template)
            reference_features: Features de referência (ou área de referência)
            
        Returns:
            estimated_depth: Profundidade estimada em metros
        """
        # Caso especial: se features são áreas (números), usa cálculo direto de SfM
        if (isinstance(current_features, (int, float)) and 
            isinstance(reference_features, (int, float))):
            return self._calculate_depth_from_area_ratio(current_features, reference_features)
        
        # Caso original: features são listas de pontos
        if reference_features is None or len(current_features) < 4 or len(reference_features) < 4:
            return None
        
        # Calcula distância média entre features para cada conjunto
        def calculate_mean_distance(features):
            if len(features) < 2:
                return 0
            distances = []
            for i in range(len(features)):
                for j in range(i + 1, len(features)):
                    dist = np.linalg.norm(np.array(features[i]) - np.array(features[j]))
                    distances.append(dist)
            return np.mean(distances) if distances else 0
        
        current_size = calculate_mean_distance(current_features)
        reference_size = calculate_mean_distance(reference_features)
        
        if current_size == 0 or reference_size == 0:
            return None
        
        # Razão de tamanhos (inversamente proporcional à profundidade)
        size_ratio = reference_size / current_size
        
        # Estimativa de profundidade relativa
        # Se as features ficaram menores, o drone se afastou (maior profundidade)
        if hasattr(self, 'reference_depth'):
            estimated_depth = self.reference_depth * size_ratio
        else:
            # Usa altitude como estimativa inicial
            estimated_depth = 5.0 * size_ratio  # Default inicial
        
        return float(max(0.5, estimated_depth))  # Profundidade mínima
    
    def _calculate_depth_from_area_ratio(self, current_area: float, reference_area: float) -> Optional[float]:
        """
        Calcula profundidade baseada na razão de áreas (SfM real)
        
        Lei física: área aparente ∝ 1/distância²
        Se área atual / área referência = k, então distância atual / distância referência = 1/√k
        
        Args:
            current_area: Área atual do template
            reference_area: Área de referência do template
            
        Returns:
            estimated_depth: Profundidade estimada
        """
        if reference_area <= 0 or current_area <= 0:
            return None
        
        # Calcula razão de áreas
        area_ratio = current_area / reference_area
        
        # Calcula fator de mudança de distância baseado na área
        # área ∝ 1/distância² → distância ∝ 1/√área
        distance_change_factor = 1.0 / math.sqrt(area_ratio)
        
        # Aplica à profundidade de referência
        if hasattr(self, 'reference_depth') and self.reference_depth > 0:
            estimated_depth = self.reference_depth * distance_change_factor
        else:
            # Se não há referência, estima baseado em altitude padrão
            estimated_depth = 5.0 * distance_change_factor
        
        # Aplica limites razoáveis
        return float(max(0.5, min(50.0, estimated_depth)))
    
    def raycast_to_ground_plane(self, u: float, v: float, altitude: float, 
                               camera_tilt_angle: float = 0.0) -> Tuple[float, float, float]:
        """
        Projeta um raio da câmera para o plano do solo
        Considera câmera frontal com possível inclinação
        
        Args:
            u, v: Coordenadas de pixel
            altitude: Altitude do drone (metros)
            camera_tilt_angle: Ângulo de inclinação da câmera (radianos, positivo = para baixo)
            
        Returns:
            x, y, z: Coordenadas 3D relativas ao drone em NED
        """
        # Converte para coordenadas normalizadas
        x_norm, y_norm = self.pixel_to_normalized_coordinates(u, v)
        
        # Para câmera frontal, precisa considerar a geometria
        # y_norm > 0 significa pixel abaixo do centro da imagem
        
        # Verifica se o pixel está na metade inferior (pode ver o chão)
        if v <= self.cy + camera_tilt_angle * self.fy:
            # Pixel está no horizonte ou acima, não pode ser projetado no solo
            return 0, 0, 0
        
        # Calcula ângulos de visão
        angle_depression = math.atan(y_norm) + camera_tilt_angle  # Ângulo para baixo
        angle_lateral = math.atan(x_norm)  # Ângulo lateral
        
        if angle_depression <= 0:
            # Não está olhando para baixo o suficiente
            return 0, 0, 0
        
        # Trigonometria para intersecção com o solo
        distance_forward = altitude / math.tan(angle_depression)
        distance_lateral = distance_forward * math.tan(angle_lateral)
        
        # Coordenadas NED relativas
        x_ned = distance_forward  # Frente
        y_ned = -distance_lateral  # Esquerda (inverte o sinal)
        z_ned = 0  # Mantém altitude
        
        return x_ned, y_ned, z_ned
    
    def enhanced_pixel_to_3d(self, u: float, v: float, 
                           current_altitude: float,
                           depth_estimate: Optional[float] = None,
                           current_features: Optional[List] = None) -> Tuple[float, float, float]:
        """
        Conversão melhorada usando múltiplas fontes de informação
        
        Args:
            u, v: Coordenadas de pixel
            current_altitude: Altitude atual do drone
            depth_estimate: Estimativa de profundidade do filtro de Kalman
            current_features: Features atuais para SfM
            
        Returns:
            x, y, z: Coordenadas 3D em NED
        """
        # Método 1: Raycast básico usando altitude
        x1, y1, z1 = self.raycast_to_ground_plane(u, v, current_altitude)
        
        # Método 2: Se temos estimativa de profundidade, usa ela
        if depth_estimate is not None and depth_estimate > 0:
            x_norm, y_norm = self.pixel_to_normalized_coordinates(u, v)
            
            # Projeta usando a profundidade estimada
            x2 = depth_estimate * x_norm
            y2 = depth_estimate * y_norm  
            z2 = 0  # Assumindo plano do solo
            
            # Combina os dois métodos com pesos
            weight_altitude = 0.3
            weight_depth = 0.7
            
            x_final = weight_altitude * x1 + weight_depth * x2
            y_final = weight_altitude * y1 + weight_depth * y2
            z_final = 0
            
            return x_final, y_final, z_final
        
        return x1, y1, z1
    
    def calculate_homography_depth(self, homography_matrix: np.ndarray, 
                                 baseline_distance: float = 1.0) -> Optional[float]:
        """
        Estima profundidade usando matriz de homografia
        Baseado na decomposição da homografia
        
        Args:
            homography_matrix: Matriz de homografia 3x3
            baseline_distance: Distância percorrida pelo drone
            
        Returns:
            estimated_depth: Profundidade estimada
        """
        if homography_matrix is None:
            return None
        
        try:
            # Decomposição da homografia para extrair movimento
            num_solutions, rotations, translations, normals = cv2.decomposeHomographyMat(
                homography_matrix, self.K)
            
            if num_solutions > 0:
                # Usa a primeira solução válida
                translation = translations[0].flatten()
                
                # A componente Z da translação está relacionada à profundidade
                if translation[2] != 0:
                    depth = baseline_distance / abs(translation[2])
                    return float(max(0.5, depth))
            
        except Exception:
            pass
        
        return None
    
    def update_reference_depth(self, depth: float):
        """
        Atualiza a profundidade de referência para comparações futuras
        """
        self.reference_depth = depth
    
    def validate_3d_coordinates(self, x: float, y: float, z: float, 
                              max_distance: float = 50.0) -> bool:
        """
        Valida se as coordenadas 3D são razoáveis
        
        Args:
            x, y, z: Coordenadas a validar
            max_distance: Distância máxima permitida
            
        Returns:
            valid: True se as coordenadas são válidas
        """
        distance = math.sqrt(x*x + y*y)
        
        # Verifica limites razoáveis
        if distance > max_distance:
            return False
        if abs(z) > 2.0:  # Não deveria ter componente Z significativa para solo
            return False
        
        return True


class AdaptiveCameraCalibration:
    """
    Calibração adaptativa da câmera baseada em features do ambiente
    Melhora os parâmetros intrínsecos durante o voo
    """
    
    def __init__(self, initial_camera_matrix, image_size):
        """
        Inicializa a calibração adaptativa
        """
        self.K = initial_camera_matrix.copy()
        self.image_size = image_size
        
        # Histórico de calibrações
        self.calibration_history = []
        
    def refine_calibration(self, image_points, world_points=None):
        """
        Refina a calibração usando pontos detectados
        
        Args:
            image_points: Pontos na imagem
            world_points: Pontos correspondentes no mundo (se conhecidos)
        """
        # Implementação de refinamento adaptativo
        # Por enquanto, mantém os parâmetros originais
        pass
    
    def get_current_camera_matrix(self):
        """
        Retorna a matriz de câmera atual (possivelmente refinada)
        """
        return self.K

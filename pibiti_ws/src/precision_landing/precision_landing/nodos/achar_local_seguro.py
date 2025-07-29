import py_trees
import numpy as np
import cv2
import threading
import time

# Importação condicional de GzCam
try:
    from drone_behaviors.camera.camera import GzCam
    gzcamera_available = True
except ImportError:
    gzcamera_available = False


class achar_local_seguro(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.commander = commander
        self.node = None
        self.local_seguro_encontrado = False
        
        # Variáveis para estabilização pós-takeoff
        self.stabilization_phase = True
        self.stabilization_start_time = None
        self.stabilization_duration = 5.0  # 5 segundos
        self.best_landing_spot = None
        self.best_landing_score = float('inf')
        
        # Compatibilidade com outros nodos
        self.camera_pronta = False
        
        
    def setup(self, **kwargs):
        """Configuração inicial do nodo"""
        try:
            # Usar o commander como node
            self.node = self.commander
            self.camera_pronta = True  # Assume que camera_feed está ativo
            
            self.logger.info("Achar local seguro configurado - usando camera_feed")
            return True
            
        except Exception as e:
            self.logger.error(f"Falha na configuração: {str(e)}")
            return False
        
    def initialise(self):
        self.logger.info("Iniciando busca por local seguro para pouso")
    
    def analisar_local_seguro(self, image):
        """
        Analisa a imagem para encontrar um local seguro para pouso usando detecção de features
        Considera apenas pontos com Y >= 480 (metade inferior da imagem onde o chão deve estar)
        Retorna (best_position, best_score) se encontrou um local adequado, (None, None) caso contrário
        """
        try:
            # Converte para escala de cinza
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            
            # Dimensões da imagem completa
            h, w = gray.shape
            block_size = 100  # Tamanho do bloco em pixels
            
            best_score = float('inf')  # Menor número de features = melhor
            best_position = None
            
            # Inicializa detector de features
            fast = cv2.FastFeatureDetector_create(threshold=30)
            
            # Percorre toda a imagem, mas só considera blocos com centro Y >= 480
            for y in range(0, h - block_size, block_size // 2):
                for x in range(0, w - block_size, block_size // 2):
                    # Calcula o centro do bloco
                    center_y = y + block_size // 2
                    center_x = x + block_size // 2
                    
                    # Só considera blocos na metade inferior (Y >= 480)
                    if center_y < 480:
                        continue
                    
                    # Extrai o bloco da imagem
                    block = gray[y:y + block_size, x:x + block_size]
                    
                    # Detecta features no bloco
                    keypoints = fast.detect(block, None)
                    feature_count = len(keypoints)
                    
                    # Calcula score baseado na densidade de features
                    # Considera tanto o número de features quanto a variação de intensidade
                    intensity_variance = np.var(block)
                    combined_score = feature_count + (intensity_variance * 0.01)  # Peso para variação
                    
                    # Menor score = área mais plana (menos features)
                    if combined_score < best_score:
                        best_score = combined_score
                        # Posição já está nas coordenadas da imagem completa
                        best_position = (center_x, center_y)
            
            if best_position is not None:
                # Durante a estabilização, atualiza o melhor local se encontrar um melhor
                if self.stabilization_phase:
                    if best_score < self.best_landing_score:
                        self.best_landing_spot = best_position
                        self.best_landing_score = best_score
                        if self.node:
                            self.node.get_logger().info(f"Novo melhor local durante estabilização: {best_position}, score: {best_score:.2f}")
                else:
                    # Após estabilização, salva no blackboard normalmente (se necessário)
                    blackboard = py_trees.blackboard.Blackboard()
                    blackboard.set("local_seguro_pixel", best_position)
                    blackboard.set("local_seguro_score", best_score)
                    blackboard.set("current_image", image)  # Salva imagem para outros nodos se necessário
                    blackboard.set("centro_area_plana", best_position)  # Centro da área com menos features
                    
                    if self.node:
                        self.node.get_logger().info(f"Local seguro encontrado em pixel: {best_position}, features score: {best_score:.2f}")
                
                return best_position, best_score
            else:
                return None, None
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro ao analisar local seguro: {e}")
            return None, None
        
    def update(self):
        """
        Comportamento que analisa imagens do camera_feed para encontrar local seguro
        """
        try:
            blackboard = py_trees.blackboard.Blackboard()
            
            # Verifica se há imagem disponível do camera_feed
            if not blackboard.exists("current_image"):
                self.logger.warning("Imagem não disponível do camera_feed")
                return py_trees.common.Status.RUNNING
            
            image = blackboard.get("current_image")
            if image is None:
                return py_trees.common.Status.RUNNING
            
            # Fase de estabilização: aguarda 5 segundos após takeoff
            if self.stabilization_phase:
                if self.stabilization_start_time is None:
                    self.stabilization_start_time = time.time()
                    if self.node:
                        self.node.get_logger().info("Iniciando fase de estabilização (5s) - procurando melhor local...")
                
                elapsed_time = time.time() - self.stabilization_start_time
                
                if elapsed_time < self.stabilization_duration:
                    # Ainda na fase de estabilização, continua buscando
                    best_position, best_score = self.analisar_local_seguro(image)
                    
                    # Visualiza a imagem com as análises
                    self.display_analysis_image(image, best_position, best_score)
                    
                    if best_position is not None:
                        # Durante a estabilização, atualiza o melhor local se encontrar um melhor
                        if best_score < self.best_landing_score:
                            self.best_landing_spot = best_position
                            self.best_landing_score = best_score
                            if self.node:
                                self.node.get_logger().info(f"Novo melhor local durante estabilização: {best_position}, score: {best_score:.2f}")
                    
                    return py_trees.common.Status.RUNNING
                else:
                    # Fase de estabilização concluída
                    self.stabilization_phase = False
                    if self.best_landing_spot is not None:
                        # Salva o melhor local encontrado no blackboard
                        blackboard.set("local_seguro_pixel", self.best_landing_spot)
                        blackboard.set("local_seguro_score", self.best_landing_score)
                        
                        if self.node:
                            self.node.get_logger().info(f"Estabilização concluída! Melhor local fixado: {self.best_landing_spot}, score: {self.best_landing_score:.2f}")
                        return py_trees.common.Status.SUCCESS
                    else:
                        if self.node:
                            self.node.get_logger().warn("Nenhum local seguro encontrado durante estabilização")
                        return py_trees.common.Status.FAILURE
            
            # Após estabilização, mantém o local fixo
            return py_trees.common.Status.SUCCESS
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro no update do achar_local_seguro: {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            if self.node:
                self.node.get_logger().info("Local seguro encontrado com sucesso")
        else:
            if self.node:
                self.node.get_logger().info("Busca por local seguro finalizada")

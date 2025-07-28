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
        self.cam = None
        self.camera_topic = "/camera"
        self.camera_resolution = (1280, 960)
        self.camera_pronta = False
        self.running = False
        self.thread = None
        self.node = None
        self.local_seguro_encontrado = False
        
        # Variáveis para estabilização pós-takeoff
        self.stabilization_phase = True
        self.stabilization_start_time = None
        self.stabilization_duration = 5.0  # 5 segundos
        self.best_landing_spot = None
        self.best_landing_score = float('inf')
        
        
    def setup(self, **kwargs):
        """Configuração inicial do nodo"""
        try:
            # Usar o commander como node (não depende do parâmetro kwargs)
            self.node = self.commander
                
            # Inicializar GzCam diretamente
            if not gzcamera_available:
                self.logger.error("GzCam não disponível!")
                return False
                
            self.cam = GzCam(self.camera_topic, self.camera_resolution)
            self.logger.info("GzCam inicializada com sucesso para busca de local seguro")
            
            # Iniciar thread de processamento contínuo
            self.running = True
            self.thread = threading.Thread(target=self.process_camera, daemon=True)
            self.thread.start()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Falha na inicialização da câmera: {str(e)}")
            self.camera_pronta = False
            return False
        
    def initialise(self):
        self.logger.info("Iniciando busca por local seguro para pouso")
        
    def process_camera(self):
        """Processa continuamente as imagens da câmera em uma thread separada"""
        while self.running:
            try:
                if self.cam is None:
                    self.logger.warning("Câmera não inicializada, aguardando...")
                    
                # Captura nova imagem da câmera Gazebo
                image = self.cam.get_next_image(timeout=1.0)
                
                if image is None:
                    self.logger.warning("Imagem da câmera não capturada, tentando novamente...")
                    time.sleep(0.1)
                    continue
                
                # Desenha linha divisória para mostrar área de análise (metade inferior)
                h, w = image.shape[:2]
                meio_altura = h // 2
                cv2.line(image, (0, meio_altura), (w, meio_altura), (255, 255, 0), 2)  # Linha amarela
                cv2.putText(image, 'Area de Analise', 
                           (10, meio_altura + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Análise para encontrar local seguro
                best_position, best_score = self.analisar_local_seguro(image)
                
                if best_position is not None:
                    self.local_seguro_encontrado = True
                    self.camera_pronta = True
                    
                    # Marca o melhor local na imagem (centro da área com menos features)
                    cv2.circle(image, best_position, 25, (0, 255, 0), 3)  # Círculo verde maior
                    cv2.circle(image, best_position, 8, (0, 255, 0), -1)   # Centro preenchido maior
                    cv2.putText(image, f'Melhor Local Pouso', 
                               (best_position[0] + 30, best_position[1]), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(image, f'Score: {best_score:.1f}', 
                               (best_position[0] + 30, best_position[1] + 25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # Desenha um retângulo ao redor da área selecionada
                    block_size = 100
                    top_left = (best_position[0] - block_size//2, best_position[1] - block_size//2)
                    bottom_right = (best_position[0] + block_size//2, best_position[1] + block_size//2)
                    cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
                
                # Exibe a imagem da câmera usando cv2.imshow
                cv2.imshow("Camera Gazebo - Local Seguro", image)
                cv2.waitKey(1)  # Necessário para atualizar a janela
                    
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Erro no processamento da câmera: {e}")
                time.sleep(1)
    
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
        Comportamento contínuo que roda em paralelo com outros nodos
        """
        try:
            # Verifica se a câmera está pronta e funcionando
            if not self.camera_pronta:
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
                    return py_trees.common.Status.RUNNING
                else:
                    # Fase de estabilização concluída
                    self.stabilization_phase = False
                    if self.best_landing_spot is not None:
                        # Salva o melhor local encontrado no blackboard
                        blackboard = py_trees.blackboard.Blackboard()
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
        # Para a thread de processamento
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
            
        # Fecha a janela do cv2 quando o nodo é finalizado
        cv2.destroyWindow("Camera Gazebo - Local Seguro")
        
        if new_status == py_trees.common.Status.SUCCESS:
            if self.node:
                self.node.get_logger().info("Local seguro encontrado com sucesso")
        else:
            if self.node:
                self.node.get_logger().info("Busca por local seguro finalizada")

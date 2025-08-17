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
        self.camera_topic = "/world/forest/model/x500_mono_cam_0/link/camera_link/sensor/imager/image"
        self.camera_resolution = (1280, 960) 
        self.camera_pronta = False
        self.running = False
        self.thread = None
        self.node = None
        self.local_seguro_encontrado = False
        
        # Variáveis para estabilização pós-takeoff
        self.stabilization_phase = False  # Inicia False, só ativa após takeoff
        self.stabilization_start_time = None
        self.stabilization_duration = 5.0  # 5 segundos
        self.best_landing_spot = None
        self.best_landing_score = float('inf')
        self.takeoff_completed = False  # Flag para verificar se takeoff foi concluído
        
        # Variáveis para rastreamento após escolha do local
        self.local_fixado = False
        self.local_fixado_pixel = None
        self.tracking_enabled = False
        
        
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
                    time.sleep(0.5)
                    continue
                    
                # Captura nova imagem da câmera Gazebo
                image = self.cam.get_next_image(timeout=1.0)
                
                if image is None:
                    time.sleep(0.1)
                    continue
                
                # Verificação e correção das dimensões da imagem (sem logs frequentes)
                h, w = image.shape[:2]
                if h != self.camera_resolution[1] or w != self.camera_resolution[0]:
                    # Redimensiona para a resolução esperada
                    image = cv2.resize(image, self.camera_resolution)
                    h, w = image.shape[:2]
                
                meio_altura = h // 2
                cv2.line(image, (0, meio_altura), (w, meio_altura), (255, 255, 0), 2)  # Linha amarela
                cv2.putText(image, 'Area de Analise', 
                           (10, meio_altura + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Status da missão na imagem
                if not self.takeoff_completed:
                    cv2.putText(image, 'Aguardando Takeoff...', 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                elif self.stabilization_phase:
                    cv2.putText(image, 'Estabilizando - Buscando Local...', 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                elif self.local_fixado:
                    cv2.putText(image, 'Rastreando Local Fixado', 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Se o local já foi fixado, apenas rastreia
                if self.local_fixado and self.local_fixado_pixel is not None:
                    # Rastreia o local fixado
                    tracked_position = self.rastrear_local_fixado(image, self.local_fixado_pixel)
                    if tracked_position is not None:
                        # Atualiza posição rastreada no blackboard
                        blackboard = py_trees.blackboard.Blackboard()
                        blackboard.set("local_seguro_pixel", tracked_position)
                        
                        # Marca o local rastreado na imagem
                        cv2.circle(image, tracked_position, 25, (0, 0, 255), 3)  # Círculo vermelho (rastreado)
                        cv2.circle(image, tracked_position, 8, (0, 0, 255), -1)   # Centro preenchido
                        cv2.putText(image, f'Local Rastreado', 
                                   (tracked_position[0] + 30, tracked_position[1]), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                elif self.stabilization_phase:  # Só analisa durante estabilização
                    # Análise para encontrar local seguro (apenas durante estabilização)
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
                else:
                    # Apenas mostra a imagem sem análise (aguardando takeoff)
                    self.camera_pronta = True  # Marca câmera como pronta mesmo sem análise
                
                # Exibe a imagem da câmera usando cv2.imshow
                cv2.imshow("Camera Gazebo - Local Seguro", image)
                cv2.waitKey(1)  # Necessário para atualizar a janela
                    
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Erro no processamento da câmera: {e}")
                time.sleep(1)
    
    def rastrear_local_fixado(self, image, local_fixado):
        """
        Rastreia o local fixado usando uma pequena área ao redor da posição original
        """
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            h, w = gray.shape
            
            # Área de busca ao redor do local fixado (±50 pixels)
            search_radius = 50
            x_center, y_center = local_fixado
            
            x_min = max(0, x_center - search_radius)
            x_max = min(w, x_center + search_radius)
            y_min = max(0, y_center - search_radius) 
            y_max = min(h, y_center + search_radius)
            
            # Extrai região de interesse
            roi = gray[y_min:y_max, x_min:x_max]
            
            if roi.shape[0] < 20 or roi.shape[1] < 20:
                return local_fixado  # Retorna posição original se ROI muito pequena
            
            # Detecta features na ROI
            fast = cv2.FastFeatureDetector_create(threshold=30)
            keypoints = fast.detect(roi, None)
            
            if len(keypoints) > 0:
                # Encontra o centroide das features
                x_sum = sum([kp.pt[0] for kp in keypoints])
                y_sum = sum([kp.pt[1] for kp in keypoints]) 
                centroid_x = int(x_sum / len(keypoints)) + x_min
                centroid_y = int(y_sum / len(keypoints)) + y_min
                
                # Verifica se o centroide está dentro dos limites
                if 0 <= centroid_x < w and 0 <= centroid_y < h:
                    return (centroid_x, centroid_y)
            
            # Se não encontrou features suficientes, mantém posição original
            return local_fixado
            
        except Exception:
            return local_fixado  # Em caso de erro, mantém posição original
    
    def analisar_local_seguro(self, image):
        """
        Analisa a imagem para encontrar um local seguro para pouso usando detecção de features
        Considera apenas pontos com Y >= metade da altura (metade inferior da imagem onde o chão deve estar)
        Retorna (best_position, best_score) se encontrou um local adequado, (None, None) caso contrário
        """
        try:
            # Converte para escala de cinza
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            
            # Dimensões da imagem atual (pode variar da configuração inicial)
            h, w = gray.shape
            meio_altura = h // 2  # Calcula dinamicamente a metade da altura
            block_size = 100  # Tamanho do bloco em pixels
            
            best_score = float('inf')  # Menor número de features = melhor
            best_position = None
            
            # Inicializa detector de features
            fast = cv2.FastFeatureDetector_create(threshold=30)
            
            # Percorre apenas a metade inferior da imagem com bounds corretos
            y_start = meio_altura  # Começa na metade da imagem
            y_end = h - block_size  # Para quando não há mais espaço para um bloco completo
            x_end = w - block_size  # Para quando não há mais espaço para um bloco completo
            
            if y_end <= y_start or x_end <= 0:
                return None, None
            
            blocks_analyzed = 0
            
            # Percorre a metade inferior da imagem
            for y in range(y_start, y_end, block_size // 2):
                for x in range(0, x_end, block_size // 2):
                    # Calcula o centro do bloco
                    center_y = y + block_size // 2
                    center_x = x + block_size // 2
                    
                    # Verifica se o centro está dentro dos limites da imagem
                    if center_x >= w or center_y >= h:
                        continue
                    
                    # Extrai o bloco da imagem com verificação de bounds
                    block_y_end = min(y + block_size, h)
                    block_x_end = min(x + block_size, w)
                    block = gray[y:block_y_end, x:block_x_end]
                    
                    # Pula blocos muito pequenos
                    if block.shape[0] < block_size//2 or block.shape[1] < block_size//2:
                        continue
                    
                    blocks_analyzed += 1
                    
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
                # Validação final das coordenadas
                if best_position[0] >= w or best_position[1] >= h:
                    return None, None
                
                # Durante a estabilização, atualiza o melhor local se encontrar um melhor
                if self.stabilization_phase:
                    if best_score < self.best_landing_score:
                        # Log reduzido - apenas quando encontra um local significativamente melhor
                        if self.node and (self.best_landing_score == float('inf') or best_score < self.best_landing_score * 0.8):
                            self.node.get_logger().info(f"Novo melhor local: {best_position}, score: {best_score:.2f}")
                        
                        self.best_landing_spot = best_position
                        self.best_landing_score = best_score
                else:
                    # Após estabilização, salva no blackboard normalmente (se necessário)
                    blackboard = py_trees.blackboard.Blackboard()
                    blackboard.set("local_seguro_pixel", best_position)
                    blackboard.set("local_seguro_score", best_score)
                    blackboard.set("current_image", image)  # Salva imagem para outros nodos se necessário
                    blackboard.set("centro_area_plana", best_position)  # Centro da área com menos features
                
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
            
            # Verifica se o takeoff foi concluído através do blackboard
            blackboard = py_trees.blackboard.Blackboard()
            
            # Verifica se existe indicador de takeoff completo no blackboard
            if not self.takeoff_completed:
                # Possíveis indicadores de takeoff completo:
                # - blackboard.exists("takeoff_completed") 
                # - blackboard.exists("flight_mode") and blackboard.get("flight_mode") == "OFFBOARD"
                # - Verificar altitude mínima
                
                takeoff_done = False
                
                # Método 1: Verifica se existe flag específica
                if blackboard.exists("takeoff_completed"):
                    takeoff_done = blackboard.get("takeoff_completed")
                
                # Método 2: Verifica modo de voo
                elif blackboard.exists("flight_mode"):
                    flight_mode = blackboard.get("flight_mode")
                    if flight_mode == "OFFBOARD" or flight_mode == "AUTO.LOITER":
                        takeoff_done = True
                
                # Método 3: Fallback - verifica se existe posição e altitude mínima
                elif blackboard.exists("current_position"):
                    pos = blackboard.get("current_position")
                    if hasattr(pos, 'z') and abs(pos.z) > 2.0:  # Altitude mínima de 2 metros
                        takeoff_done = True
                
                if takeoff_done:
                    self.takeoff_completed = True
                    self.stabilization_phase = True  # Inicia estabilização
                    if self.node:
                        self.node.get_logger().info("✈️ Takeoff detectado! Iniciando estabilização...")
                else:
                    # Ainda aguardando takeoff
                    return py_trees.common.Status.RUNNING
            
            # Fase de estabilização: aguarda 5 segundos após takeoff completo
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
                        # Fixa o melhor local encontrado
                        self.local_fixado = True
                        self.local_fixado_pixel = self.best_landing_spot
                        self.tracking_enabled = True
                        
                        # Salva o melhor local encontrado no blackboard
                        blackboard.set("local_seguro_pixel", self.best_landing_spot)
                        blackboard.set("local_seguro_score", self.best_landing_score)
                        
                        if self.node:
                            self.node.get_logger().info(f"🎯 LOCAL FIXADO! Posição: {self.best_landing_spot}, Score: {self.best_landing_score:.2f}")
                            self.node.get_logger().info("Iniciando rastreamento do local escolhido...")
                        
                        return py_trees.common.Status.RUNNING
                    else:
                        if self.node:
                            self.node.get_logger().warn("Nenhum local seguro encontrado durante estabilização")
                        return py_trees.common.Status.FAILURE
            
            # Após estabilização, continua rodando com rastreamento ativo
            return py_trees.common.Status.RUNNING
                
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

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
                    time.sleep(0.1)
                    continue
                    
                # Captura nova imagem da câmera Gazebo
                image = self.cam.get_next_image(timeout=1.0)
                
                if image is None:
                    continue
                
                # Cria uma cópia da imagem para visualização
                display_image = image.copy()
                
                # Análise para encontrar local seguro
                best_position, best_score = self.analisar_local_seguro(image)
                
                if best_position is not None:
                    self.local_seguro_encontrado = True
                    self.camera_pronta = True
                    
                    # Marca o melhor local na imagem
                    cv2.circle(display_image, best_position, 20, (0, 255, 0), 3)  # Círculo verde
                    cv2.circle(display_image, best_position, 5, (0, 255, 0), -1)   # Centro preenchido
                    cv2.putText(display_image, f'Local Seguro', 
                               (best_position[0] + 25, best_position[1]), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(display_image, f'Score: {best_score:.2f}', 
                               (best_position[0] + 25, best_position[1] + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Exibe a imagem da câmera usando cv2.imshow
                cv2.imshow("Camera Gazebo - Local Seguro", display_image)
                cv2.waitKey(1)  # Necessário para atualizar a janela
                    
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Erro no processamento da câmera: {e}")
                time.sleep(1)
    
    def analisar_local_seguro(self, image):
        """
        Analisa a imagem para encontrar um local seguro para pouso
        Retorna (best_position, best_score) se encontrou um local adequado, (None, None) caso contrário
        """
        try:
            # Procura por área com baixa variação de cor (área plana)
            # Converte para escala de cinza
            gray = np.mean(image, axis=2) if len(image.shape) == 3 else image
            
            # Divide a imagem em blocos e analisa a variação
            h, w = gray.shape
            block_size = 50  # Tamanho do bloco em pixels
            
            best_score = float('inf')
            best_position = None
            
            for y in range(0, h - block_size, block_size // 2):
                for x in range(0, w - block_size, block_size // 2):
                    block = gray[y:y + block_size, x:x + block_size]
                    
                    # Calcula a variação no bloco (desvio padrão)
                    variation = np.std(block)
                    
                    # Menor variação = área mais plana/segura
                    if variation < best_score:
                        best_score = variation
                        best_position = (x + block_size // 2, y + block_size // 2)  # Centro do bloco
            
            if best_position is not None:
                # Armazena a posição do local seguro no blackboard
                blackboard = py_trees.blackboard.Blackboard()
                blackboard.set("local_seguro_pixel", best_position)
                blackboard.set("local_seguro_score", best_score)
                blackboard.set("current_image", image)  # Salva imagem para outros nodos se necessário
                
                if self.node:
                    self.node.get_logger().info(f"Local seguro encontrado em pixel: {best_position}, score: {best_score:.2f}")
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
        Similar ao exemplo de detecção fornecido
        """
        try:
            # Verifica se a câmera está pronta e funcionando
            if not self.camera_pronta:
                return py_trees.common.Status.RUNNING
                
            # Behavior roda continuamente, mantendo o estado RUNNING
            # A análise real é feita na thread separada
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

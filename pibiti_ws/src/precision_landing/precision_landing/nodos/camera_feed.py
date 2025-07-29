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


class camera_feed(py_trees.behaviour.Behaviour):
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
        
        # Variáveis para análise de imagem (transferidas do achar_local_seguro)
        self.current_image = None
        self.image_lock = threading.Lock()
        
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
            self.logger.info("GzCam inicializada com sucesso para camera_feed")
            
            # Teste de OpenCV para display
            try:
                # Cria uma imagem de teste pequena
                test_img = np.zeros((100, 100, 3), dtype=np.uint8)
                cv2.putText(test_img, 'TEST', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow("Camera Feed - Sistema de Precisao", test_img)
                cv2.waitKey(1)
                self.logger.info("Teste OpenCV: Janela criada com sucesso")
            except Exception as opencv_error:
                self.logger.error(f"Erro no teste OpenCV: {opencv_error}")
                self.logger.warning("Display pode não funcionar - continuando sem visualização")
            
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
        self.logger.info("Iniciando feed contínuo da câmera")
        
    def process_camera(self):
        """Processa continuamente as imagens da câmera em uma thread separada"""
        while self.running:
            try:
                if self.cam is None:
                    self.logger.warning("Câmera não inicializada, aguardando...")
                    time.sleep(1)
                    continue
                    
                # Captura nova imagem da câmera Gazebo
                image = self.cam.get_next_image(timeout=1.0)
                
                if image is None:
                    self.logger.warning("Imagem da câmera não capturada, tentando novamente...")
                    time.sleep(0.1)
                    continue
                
                # Thread-safe: atualiza imagem atual
                with self.image_lock:
                    self.current_image = image.copy()
                    self.camera_pronta = True
                
                # Atualiza blackboard com a imagem atual
                blackboard = py_trees.blackboard.Blackboard()
                blackboard.set("current_image", image)
                blackboard.set("camera_pronta", True)
                
                # Visualização da imagem com informações básicas
                display_image = image.copy()
                
                # Desenha linha divisória para mostrar área de análise (metade inferior)
                h, w = display_image.shape[:2]
                meio_altura = h // 2
                cv2.line(display_image, (0, meio_altura), (w, meio_altura), (255, 255, 0), 2)  # Linha amarela
                cv2.putText(display_image, 'Area de Analise', 
                           (10, meio_altura + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Marca o centro da imagem
                center_x, center_y = w // 2, h // 2
                cv2.circle(display_image, (center_x, center_y), 10, (0, 0, 255), 2)  # Centro vermelho
                cv2.putText(display_image, 'Centro', 
                           (center_x + 15, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Informações do sistema
                cv2.putText(display_image, f'Resolucao: {w}x{h}', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_image, f'Camera: {"ON" if self.camera_pronta else "OFF"}', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.camera_pronta else (0, 0, 255), 2)
                
                # Debug: adiciona timestamp
                timestamp = time.strftime("%H:%M:%S", time.localtime())
                cv2.putText(display_image, f'Time: {timestamp}', 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Contador de frames para debug
                if not hasattr(self, 'tick_count'):
                    self.tick_count = 0
                self.tick_count += 1
                
                # Exibe a imagem da câmera
                try:
                    cv2.imshow("Camera Feed - Sistema de Precisao", display_image)
                    cv2.waitKey(1)  # Necessário para atualizar a janela
                    
                    # Debug: confirma que a imagem foi processada
                    if self.tick_count % 30 == 0:  # Log a cada 30 frames (~1.5s)
                        if self.node:
                            self.node.get_logger().info(f"Camera feed ativo - Frame {w}x{h} processado às {timestamp}")
                            
                except Exception as cv_error:
                    if self.node:
                        self.node.get_logger().error(f"Erro ao mostrar imagem OpenCV: {cv_error}")
                
                
                    
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Erro no processamento da câmera: {e}")
                time.sleep(1)
    
    def get_current_image(self):
        """Retorna a imagem atual de forma thread-safe"""
        with self.image_lock:
            return self.current_image.copy() if self.current_image is not None else None
    
    def update(self):
        """
        Comportamento contínuo que mantém o feed da câmera ativo
        """
        try:
            # Verifica se a câmera está funcionando
            if not self.camera_pronta:
                return py_trees.common.Status.RUNNING
            
            # Mantém o feed ativo continuamente
            return py_trees.common.Status.RUNNING
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro no update do camera_feed: {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        # Para a thread de processamento
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
            
        # Fecha a janela do cv2 quando o nodo é finalizado
        cv2.destroyWindow("Camera Feed - Sistema de Precisao")
        
        if new_status == py_trees.common.Status.SUCCESS:
            if self.node:
                self.node.get_logger().info("Camera feed finalizado com sucesso")
        else:
            if self.node:
                self.node.get_logger().info("Camera feed finalizado")

import py_trees
import rclpy
import cv2
import threading
import numpy as np
import os
import time
import math

from ultralytics import YOLO

# Importação condicional de GzCam
try:
    from drone_behaviors.camera.camera import GzCam
    gzcamera_available = True
except ImportError:
    gzcamera_available = False

from std_msgs.msg import Float32MultiArray

class deteccao(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.cmdr = commander
        self.running = False
        self.node = None
        self.forma_detectada = False
        self.camera_pronta = False
        self.publisher = None
        self.thread = None
        self.camera_topic = "/camera"
        self.camera_resolution = (1280, 960)
        self.cam = None
        self.last_valid_center = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.prev_valid_center = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_frame = None
        self.model = None
        self.base_threshold = 0.75  # Threshold para detecção de bases
        
        # Sistema de confiança na detecção
        self.detection_confidence = 0  # Contador de detecções consistentes
        self.confidence_threshold = 5  # Número de detecções necessárias para confiar
        self.last_detection_pos = None  # Última posição detectada
        self.position_tolerance = 50  # Tolerância em pixels para considerar mesma posição
        
        # Parâmetros para verificar alinhamento
        self.alignment_tolerance = 30  # Tolerância em pixels para considerar alinhado
        self.was_aligned = False  # Flag para indicar se estava alinhado na última detecção
        
        self.qos_profile = self.cmdr.px4_qos_profile()

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
            if not self.node:
                raise RuntimeError("No node provided to behavior!")
            
            # Carregar modelo YOLO para detecção de bases
            model_path = 'src/Missao_3/Missao_3/visao/best.pt'
            
            if not os.path.exists(model_path):
                self.node.get_logger().error(f"Modelo não encontrado: {model_path}")
                
                return False
                
            self.model = YOLO(model_path)
            self.node.get_logger().info(f"Modelo carregado com sucesso! Classes: {self.model.names}")

            # Configurar publisher para os dados de detecção
            self.publisher = self.node.create_publisher(
                Float32MultiArray,
                'CenterData',
                qos_profile=self.qos_profile
            )

            # Inicializar GzCam
            if not gzcamera_available:
               
                raise RuntimeError("GzCam não disponível!")
                
            self.cam = GzCam(self.camera_topic, self.camera_resolution)
            self.node.get_logger().info("GzCam inicializada com sucesso")

            # Iniciar thread de processamento
            self.running = True
            self.thread = threading.Thread(target=self.process_frame, daemon=True)
            self.thread.start()

            return True

        except Exception as e:
            self.node.get_logger().error(f"Falha na inicialização: {str(e)}")
            return False

    def process_frame(self):
     while self.running and rclpy.ok():
        try:
            frame = self.cam.get_next_image()
            if frame is None:
                continue

            results = self.model(frame)
            r = results[0]

            masks = getattr(r, 'masks', None)
            boxes = getattr(r, 'boxes', None)

            if boxes is None:
                processed_frame = frame
                centers = []
            else:
                result_img = frame.copy()
                overlay = result_img.copy()
                centers = []

                base_colors = [
                    (255, 0, 255), (255, 165, 0), (0, 255, 255), (255, 255, 0),
                    (128, 0, 128), (255, 20, 147), (0, 255, 127), (255, 69, 0)
                ]

                forma_desejada = "base_cruz"  # ou o índice/nome da classe desejada
                for i, box in enumerate(r.boxes):
                    conf = float(box.conf[0]) if hasattr(box, 'conf') else 0.0
                    if conf < self.base_threshold:
                        continue

                    class_id = int(box.cls[0]) if hasattr(box, 'cls') else 0
                    class_name = self.model.names[class_id] if hasattr(self.model, 'names') and class_id in self.model.names else f"base_{class_id}"

                    if class_name != forma_desejada:
                        continue

                    color = base_colors[class_id % len(base_colors)]

                    # Pegue as coordenadas da caixa
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # Máscara
                    if masks is not None and i < len(masks.data):
                        mask = masks.data[i].cpu().numpy().astype('uint8') * 255
                        mask_resized = cv2.resize(mask, (x2 - x1, y2 - y1))
                        roi = overlay[y1:y2, x1:x2]
                        if roi.shape[:2] == mask_resized.shape:
                            roi[mask_resized > 0] = (
                                0.7 * roi[mask_resized > 0] + 0.3 * np.array(color)
                            ).astype('uint8')

                    # Caixa e rótulo
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 3)
                    class_name = self.model.names[class_id] if hasattr(self.model, 'names') and class_id in self.model.names else f"base_{class_id}"
                    label = f"{class_name}: {conf:.2f}"
                    cv2.putText(overlay, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                    # Centro
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    centers.append((cx, cy))
                    cv2.circle(overlay, (cx, cy), 12, color, -1)
                    cv2.circle(overlay, (cx, cy), 12, (255, 255, 255), 2)
                    cv2.circle(overlay, (cx, cy), 4, (0, 0, 0), -1)
                    cv2.putText(overlay, 'Centro', (cx + 18, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(overlay, f'({cx},{cy})', (cx + 18, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                processed_frame = cv2.addWeighted(result_img, 0.6, overlay, 0.4, 0)

            # Publicar resultado
            msg = Float32MultiArray()
            if centers:
                self.forma_detectada = True
                xf, yf = centers[0]  # Centro da forma
                xc = self.camera_resolution[0] // 2  # Centro X da câmera
                yc = self.camera_resolution[1] // 2  # Centro Y da câmera
                
                # Calcula distância ao centro
                dist_to_center = math.sqrt((xf - xc)**2 + (yf - yc)**2)
                
                # Verifica se a detecção está próxima da última posição válida
                if self.last_detection_pos is None:
                    self.last_detection_pos = (xf, yf)
                    self.detection_confidence = 1
                else:
                    last_x, last_y = self.last_detection_pos
                    dist = math.sqrt((xf - last_x)**2 + (yf - last_y)**2)
                    
                    if dist <= self.position_tolerance:
                        self.detection_confidence += 1
                    else:
                        # Nova posição detectada, reinicia confiança
                        self.detection_confidence = 1
                        self.last_detection_pos = (xf, yf)
                
                # Verifica se está alinhado
                self.was_aligned = dist_to_center <= self.alignment_tolerance
                
                # Log para debug
                self.node.get_logger().info(f"Confiança: {self.detection_confidence}/{self.confidence_threshold}, Alinhado: {self.was_aligned}")
                
                if self.detection_confidence >= self.confidence_threshold:
                    # Detecção confiável, atualiza posição
                    msg.data = [float(xc), float(yc), float(xf), float(yf), float(len(centers))]
                    self.prev_valid_center = list(self.last_valid_center)
                    self.last_valid_center = list(msg.data)
                else:
                    # Ainda não confiável, mantém última posição válida
                    msg.data = self.last_valid_center
                
                # Desenhar visualização para debug
                confidence_color = (0, 255, 0) if self.detection_confidence >= self.confidence_threshold else (0, 165, 255)
                alignment_color = (0, 255, 0) if self.was_aligned else (0, 0, 255)
                cv2.circle(overlay, (xc, yc), 10, (255, 0, 0), 2)  # Centro da câmera em azul
                cv2.circle(overlay, (int(xf), int(yf)), 10, confidence_color, 2)  # Centro da forma
                cv2.circle(overlay, (xc, yc), self.alignment_tolerance, alignment_color, 1)  # Círculo de tolerância
                cv2.putText(overlay, f"Confiança: {self.detection_confidence}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, confidence_color, 2)
                cv2.putText(overlay, f"Alinhado: {self.was_aligned}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, alignment_color, 2)
            else:
                # Sem detecção, mas mantém estado de alinhamento se estava alinhado
                self.detection_confidence = max(0, self.detection_confidence - 1)
                msg.data = self.last_valid_center
                # Se estava alinhado e tinha confiança, continua considerando detectado
                if self.was_aligned and self.detection_confidence >= self.confidence_threshold / 2:
                    msg.data[4] = 1.0  # Mantém como detectado
                    self.node.get_logger().info("⚠️ Sem detecção mas mantendo último alinhamento")
                else:
                    msg.data[4] = 0.0
                    self.was_aligned = False

            self.publisher.publish(msg)
            processed_frame = cv2.addWeighted(result_img, 0.6, overlay, 0.4, 0)
            self.last_frame = processed_frame

            cv2.imshow("Formas Detectadas", processed_frame)
            self.node.get_logger().info("Processando frame com sucesso.")
            self.camera_pronta = True
            
            if cv2.waitKey(1) & 0xFF == 27:
                self.node.get_logger().info("Tecla ESC pressionada, encerrando ROS...")
                rclpy.shutdown()

        except Exception as e:
            self.node.get_logger().error(f"Erro no processamento: {str(e)}")
            time.sleep(1)

    def update(self):
        if not rclpy.ok():
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

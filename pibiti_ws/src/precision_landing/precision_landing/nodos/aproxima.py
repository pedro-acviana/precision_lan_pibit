import py_trees
import math
from px4_msgs.msg import VehicleCommand


class aproxima(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.commander = commander
        self.tolerance = 0.1  # Tolerância em metros para considerar que chegou ao alvo
        self.max_velocity = 2.0  # Velocidade máxima em m/s
        self.target_reached = False
        
    def setup(self):
        self.logger.debug("Configurando aproximação ao local seguro")
        
    def initialise(self):
        self.logger.info("Iniciando aproximação ao local seguro usando comandos de velocidade")
        self.target_reached = False
        
    def update(self):
        """
        Move o drone até o local seguro usando comandos de velocidade NED
        """
        try:
            blackboard = py_trees.blackboard.Blackboard()
            
            # Obtém a posição alvo em NED
            if not blackboard.exists("target_ned_position"):
                self.logger.warn("Posição alvo NED não encontrada no blackboard")
                return py_trees.common.Status.FAILURE
                
            target_ned = blackboard.get("target_ned_position")
            target_x = target_ned['x']
            target_y = target_ned['y']
            
            # Calcula a distância até o alvo (target_ned já são distâncias relativas)
            distance_x = target_x  # Distância relativa em X (Norte)
            distance_y = target_y  # Distância relativa em Y (Leste)
            total_distance = math.sqrt(distance_x**2 + distance_y**2)
            
            # Verifica se chegou ao alvo
            if total_distance < self.tolerance:
                # Para o drone
                self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
                self.logger.info(f"Alvo alcançado! Distância: {total_distance:.2f}m")
                return py_trees.common.Status.SUCCESS
            
            # Calcula velocidades proporcionais à distância
            # Normaliza as velocidades para não exceder a velocidade máxima
            if total_distance > 0:
                velocity_x = (distance_x / total_distance) * min(self.max_velocity, total_distance)
                velocity_y = (distance_y / total_distance) * min(self.max_velocity, total_distance)
            else:
                velocity_x = 0.0
                velocity_y = 0.0
            
            # Envia comando de velocidade (NED: vx=Norte, vy=Leste, vz=Baixo)
            self.commander.publish_velocity_setpoint(
                velocity_x,  # Velocidade Norte
                velocity_y,  # Velocidade Leste  
                0.0          # Velocidade vertical (manter altitude)
            )
            
            self.logger.info(f"Aproximando: dist={total_distance:.2f}m, vel=({velocity_x:.2f}, {velocity_y:.2f})")
            
            return py_trees.common.Status.RUNNING
            
        except Exception as e:
            self.logger.error(f"Erro durante aproximação: {e}")
            # Para o drone em caso de erro
            self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        # Para o drone quando o comportamento termina
        self.commander.publish_velocity_setpoint(0.0, 0.0, 0.0)
        
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Aproximação ao local seguro concluída com sucesso")
        else:
            self.logger.info("Aproximação ao local seguro finalizada")

     
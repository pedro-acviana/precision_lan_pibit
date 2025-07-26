import py_trees

from drone_behaviors.flight.arm import Arm
from drone_behaviors.flight.desarmar import Desarmar
from drone_behaviors.flight.offboard_mode import OffboardMode
from drone_behaviors.flight.takeoff import Takeoff
from drone_behaviors.flight.iniciar_pouso import IniciarPouso

from drone_behaviors.conditions.condicoes_voo import CondicoesSeguras
from drone_behaviors.conditions.esta_no_chao import EstaNoChao

from drone_behaviors.subtrees.pouso import pouso
from drone_behaviors.subtrees.iniciar_voo import iniciar_voo

class BehaviorFactory:
    """Factory for creating common drone behaviors and subtrees."""
    
    # Flight behaviors
    @staticmethod
    def create_arm(commander):
        return Arm(commander)
    
    @staticmethod
    def create_desarmar(commander):
        return Desarmar("Desarmar",commander)
    
    @staticmethod
    def create_offboard_mode(commander):
        return OffboardMode(commander)
        
    @staticmethod
    def create_takeoff(commander, altitude=-2.0):
        return Takeoff(commander, altitude)
    
    @staticmethod
    def create_iniciar_pouso(commander):
        return IniciarPouso("Iniciar Pouso",commander)
    
    # Conditions
    @staticmethod
    def create_condicoes_seguras(commander, camera_detect):
        return CondicoesSeguras(commander, camera_detect)
    
    @staticmethod
    def create_esta_no_chao(commander):
        return EstaNoChao("Está no Chão?",commander)
    
    # Complete subtrees
    @staticmethod
    def create_landing_sequence(commander):
        return pouso(commander)
    
    @staticmethod
    def create_takeoff_sequence(commander, camera_detect):
        return iniciar_voo(commander, camera_detect)
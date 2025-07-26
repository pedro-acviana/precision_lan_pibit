import py_trees
from drone_behaviors.flight.iniciar_pouso import IniciarPouso
from drone_behaviors.conditions.esta_no_chao import EstaNoChao
from drone_behaviors.flight.desarmar import Desarmar

def pouso(commander, name="Pouso Final"):
    """Creates a standard landing sequence subtree"""
    seq = py_trees.composites.Sequence(name=name, memory=True)
    seq.add_children([
        IniciarPouso("Iniciar Pouso", commander),
        EstaNoChao("Está no Chão?", commander),
        Desarmar("Desarmar", commander)
    ])
    return seq
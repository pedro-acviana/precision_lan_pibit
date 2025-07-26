import py_trees
from drone_behaviors.conditions.condicoes_voo import CondicoesSeguras
from drone_behaviors.flight.arm import Arm
from drone_behaviors.flight.offboard_mode import OffboardMode
from drone_behaviors.flight.takeoff import Takeoff

def iniciar_voo(commander, camera_detect):
    seq = py_trees.composites.Sequence(name="iniciar voo", memory=True)
    seq.add_children([
        CondicoesSeguras(commander, camera_detect),
        py_trees.composites.Sequence(name="Armagem", memory=True, children=[
            Arm(commander, camera_detect),
            OffboardMode(commander),
            Takeoff(commander, commander.target_altitude)
        ])
    ])
    return seq
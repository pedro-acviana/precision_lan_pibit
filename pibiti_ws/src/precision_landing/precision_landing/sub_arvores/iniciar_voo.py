import py_trees
from drone_behaviors.behavior_factory import BehaviorFactory

def iniciar_voo(commander):
    """
    Create the takeoff sequence using the common behavior factory.
    This is a wrapper that uses the factory to create the standardized takeoff sequence.
    """
    # Simply use the factory to create the common takeoff sequence
    return BehaviorFactory.create_takeoff_sequence(commander)
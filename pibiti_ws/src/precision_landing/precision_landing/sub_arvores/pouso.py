import py_trees
from drone_behaviors.behavior_factory import BehaviorFactory

def pouso_final(commander):
    """
    Create the landing sequence using the common behavior factory.
    This is a wrapper that uses the factory to create the standardized landing sequence.
    """
    # Simply use the factory to create the common landing sequence
    return BehaviorFactory.create_landing_sequence(commander)
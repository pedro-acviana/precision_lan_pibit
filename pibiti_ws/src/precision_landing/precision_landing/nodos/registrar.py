import py_trees
from px4_msgs.msg import VehicleCommand

class registrar(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.cmdr = commander

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        cores_pilha = blackboard.get("cores_pilha")
        print("ANTES DO POP:", cores_pilha)
        if cores_pilha:
            cor_removida = cores_pilha.pop(0)
            print(f"Cor {cor_removida} removida da pilha!")
            print("DEPOIS DO POP:", cores_pilha)
        else:
            print("Pilha de cores já está vazia.")
        return py_trees.common.Status.SUCCESS

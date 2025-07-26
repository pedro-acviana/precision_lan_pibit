import py_trees
import time




class EstaNoChao(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.cmdr = commander

        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):       
        # Calcula o tempo decorrido desde o envio do comando
        elapsed = time.time() - self.start_time

        if  elapsed > 3.0:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

        

     
import py_trees

class rotaciona(py_trees.behaviour.Behaviour):
    def __init__(self, name, commander):
        super().__init__(name)
        self.cmdr = commander
        self.yaw_rate = 2 * 3.1416 / 20  # ~0.628 rad/s → uma volta a cada 20s
        self.altura = self.cmdr.target_altitude * -1

    def initialise(self):
        self.cmdr.get_logger().info("Iniciando rotação contínua")

    def update(self):
        # Envia setpoint com velocidade angular de yaw
        self.cmdr.muda_yaw(self.yaw_rate, self.altura)

        return py_trees.common.Status.RUNNING

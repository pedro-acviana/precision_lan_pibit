import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleLocalPosition, HomePosition, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry 
from std_msgs.msg import Float32MultiArray


class PX4Commander(Node):
    def __init__(self):
        super().__init__('mission_node')  # Match name in launch file
        qos = QoSProfile(depth=10)
        
        # Declare all parameters
        self.declare_parameter('forma', 'circle')
        self.declare_parameter('cor', [255, 0, 0])
        self.declare_parameter('nome_cor', 'red')
        self.declare_parameter('range_H_minus', 10)
        self.declare_parameter('range_H_plus', 5)
        self.declare_parameter('range_S_minus', 10)
        self.declare_parameter('range_S_plus', 5)
        self.declare_parameter('range_V_minus', 10)
        self.declare_parameter('range_V_plus', 5)
        self.declare_parameter('altitude_de_cruzeiro', 10)
        self.declare_parameter('altitude_drone', 20)
        self.declare_parameter('velocity', 5)
        
        #mission 1 parameters
        self.declare_parameter('sequence_colors', [''])
        self.declare_parameter('sequence_directions', [''])
        self.sequence = self.get_mission1_sequence()

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)
        self.local_velocity = [0.0, 0.0, 0.0]
        self.curr_position = [0.0, 0.0, 0.0]
        self.center_data_sub = self.create_subscription(
            Float32MultiArray, "/center_data", self.center_data_callback, 10
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,         
            qos
        )
        self.home_position_sub = self.create_subscription(
        HomePosition,
        '/fmu/out/home_position',
        self.home_position_callback,
        qos
        )   
        self.home_position = [0.0, 0.0, 0.0]
        self.arming_state = None
        self.nav_state = None
        self.landed_state = None
        self.current_altitude = None


        self.current_position = [0.0, 0.0, 0.0]
        self.center_data = None

        self.target_altitude = 2.0
        self.initial_yaw = None  # Armazena o yaw inicial do drone
        self.clock = self.get_clock()

        self.offboard_setpoint_counter = 0

        # Envia setpoints continuamente para manter o modo OFFBOARD estável
        self.setpoint_timer = self.create_timer(0.05, self.publish_loop)
        
    def get_mission1_sequence(self):
        """Get the mission1 sequence parameter as color-direction pairs"""
        try:
            colors = self.get_parameter('sequence_colors').value
            directions = self.get_parameter('sequence_directions').value

            # Create paired sequence
            sequence = []
            for i in range(min(len(colors), len(directions))):
                sequence.append([colors[i], directions[i]])

            # self.get_logger().info(f"Mission 1 Sequence: {sequence}")  # Removido
            return sequence
        except Exception as e:
            self.get_logger().error(f"Failed to get mission sequence parameter: {str(e)}")
            return []  # Return default empty list if parameter can't be accessed

        
    def status_callback(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.landed_state = msg.landed_state
        # self.get_logger().info(f"[STATUS] Arming: {msg.arming_state}, Nav: {msg.nav_state}, Landed: {msg.landed_state}")  # Removido para evitar spam 
        
    

    def local_position_callback(self, msg):
        self.local_velocity = [msg.vx, msg.vy, msg.vz]
        self.curr_position = [msg.x, msg.y, msg.z]
        
        # Captura o yaw inicial do drone na primeira leitura
        if self.initial_yaw is None:
            self.initial_yaw = msg.yaw
            self.get_logger().info(f"Yaw inicial capturado: {self.initial_yaw:.2f} rad")
        
        # self.current_position = msg.data[:3]  # Removido - causa erro

    def center_data_callback(self, msg):
        self.center_data = msg.data

    def local_position_alt__callback(self, msg):
        self.current_altitude = msg.z

    def publish_loop(self):
        self.publish_offboard_control_heartbeat_signal()
        self.publish_alt(self.target_altitude)
        self.offboard_setpoint_counter += 1
        
    def home_position_callback(self, msg):
        self.home_position = [msg.x, msg.y, msg.z]
        self.home_position_valid = True  # set this flag


    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, target_x, target_y, target_z):
        msg = TrajectorySetpoint()
        msg.position = [target_x, target_y, target_z]
        msg.velocity = [float("nan")] * 3  # ou defina uma velocidade fixa aqui se preferir
        msg.yaw = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.clock.now().nanoseconds / 1000)
        self.command_pub.publish(msg)
        
    def send_servo_command(self, servo_number, pwm_value, continuous=False, duration=5.0):
        msg = VehicleCommand()
        msg.command = 187  # COmando correto para acessar o servo
        msg.param1 = float(servo_number)
        msg.param2 = float(pwm_value)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.confirmation = 0
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)
        self.get_logger().info(f"Sent servo command: Servo {servo_number}, PWM {pwm_value}")
        
        # If continuous mode is requested, create a timer to send commands repeatedly
        if continuous:
            self.get_logger().info(f"Starting continuous servo commands for {duration} seconds")
            
            # Create attributes to track the current servo command
            self._servo_command = {
                'number': servo_number,
                'value': pwm_value,
                'start_time': self.get_clock().now(),
                'duration': duration,
                'count': 0
            }
            
            # Create and start the timer (10Hz just like your working example)
            self._servo_timer = self.create_timer(0.1, self._servo_command_callback)
            
        return True

    def _servo_command_callback(self):
        """Callback for the servo command timer"""
        if not hasattr(self, '_servo_command'):
            # Safety check - cancel timer if we don't have command data
            if hasattr(self, '_servo_timer'):
                self._servo_timer.cancel()
            return
        
        # Get current command data
        servo_number = self._servo_command['number']
        pwm_value = self._servo_command['value']
        start_time = self._servo_command['start_time']
        duration = self._servo_command['duration']
        
        # Check if we should stop
        now = self.get_clock().now()
        elapsed = (now - start_time).nanoseconds / 1e9
        
        if elapsed >= duration:
            # We're done, cancel the timer
            self.get_logger().info(f"Finished continuous servo commands after {elapsed:.1f} seconds")
            self._servo_timer.cancel()
            delattr(self, '_servo_timer')
            delattr(self, '_servo_command')
            return
        
        # Send the command again
        msg = VehicleCommand()
        msg.command = 187
        msg.param1 = float(servo_number)
        msg.param2 = float(pwm_value)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.confirmation = 0
        msg.from_external = True
        msg.timestamp = int(now.nanoseconds / 1000)
        self.command_pub.publish(msg)
        
        # Increment the counter and log occasionally
        self._servo_command['count'] += 1
        if self._servo_command['count'] % 10 == 0:  # Log every ~1 second
            self.get_logger().info(f"Continuous servo command #{self._servo_command['count']}, elapsed: {elapsed:.1f}/{duration:.1f}s")

    def publish_alt(self, z):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), -z]  # Usando posição Z alvo
        msg.velocity = [0.0, 0.0, 0.0]  # Velocidade de subida
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = float('nan')
        msg.timestamp = int(self.clock.now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def px4_qos_profile(self):
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

    def set_rtl_mode(self):
        """Set the drone to RTL (Return To Launch) mode"""
        self.get_logger().info("Setting RTL mode")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        return True
    
    def publish_velocity_setpoint(self, vx, vy, vz):
        msg = TrajectorySetpoint()
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.position = [float('nan')] * 3
        # Usa o yaw inicial para evitar rotação indesejada
        msg.yaw = self.initial_yaw if self.initial_yaw is not None else float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)
    
    def muda_yaw(self, yaw_rate, altura_constante):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), altura_constante]  # sem mover
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yaw = float('nan') 
        msg.yawspeed = yaw_rate  # velocidade angular em rad/smsg.yaw = yaw_angle
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)


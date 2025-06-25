# Introdução a Robótica
Trabalho-Robô Navegador
A tarefa é desenvolver um nodo ROS2 para controlar um rob diferencial.
 Esse nodo deve ser capaz de guiar o robô até e as coordenadas alvo 1 (x=7,y=7) e 
 2 (x=7,y=−3) noplano, sem colidir com os obstáculos do cenário.
 O erro de distância mínimo permitido entre o robô e o alvo,nas direções
 x e y, é de 0,3. Você deve utilizar o sensor laser para detectar e desviar dos obstáculos.
 Ambos os alvos devem ser alcançados no mesmo código.
 Você deve utilizar a posição do robô, cuja coordenada inicial é (x=−7,y=−7),
 fornecida pelo tópico/odom e as informações de distância capturadas pelo laser,
 publicadas no tópico/basescan. O controle do robô deve ser realizado por meio do envio
 de mensagens para o tópico/cmdvel, contendo a velocidade linear e angular do robô.
 Essas velocidades podem ser tratadas podem ser tratadas de forma discreta ou contínua,
 sendo a discretização uma abordagem mais simples para implementar o algoritmo de
 controle.
 Na sua implementação da tarefa, é fundamental sempre fazer a Matriz de
 Rotação e definir a posição inicial do robô no mapa,afim de evitar erros de localização.
 _______________________________________________________________________________________
Link do meu trabalho You Tube:
https://www.youtube.com/watch?v=ZurpNpvGLrQ
 _______________________________________________________________________________________
 Código:
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
# Variáveis globais
cmd_publisher = None
red_robot = None
laser_readings = []
position = [None, None]  # [x, y]
orientation = None       # yaw
# Lista de alvos (x, y)
waypoints = [
    (-1.5, -5.8), (-1.1, -0.5), (-2.3, 2.8), (6.0, 3.9), (6.5, 6.5),
    (6.3, 3.2), (1.5, 3.9), (-3.1, 2.2), (-3.1, 2.2), (-1.5, -2.3), (6.5, -2.9)
]
def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
def detect_obstacle(threshold=0.3):
    if not laser_readings:
        return False
    for distance in laser_readings:
        if distance != float('inf') and not math.isnan(distance) and distance < threshold:
            return True
    return False
def odometry_ground_callback(msg):
    global position, orientation
    position[0] = msg.pose.pose.position.x
    position[1] = msg.pose.pose.position.y
    orientation = quaternion_to_yaw(msg.pose.pose.orientation)
def odometry_callback(msg):
    # Pode ser utilizado futuramente para comparação com ground truth
    pass
def laser_scan_callback(msg):
    global laser_readings
    center = int((len(msg.ranges) - 1) / 2)
    # Evitar IndexError caso msg.ranges tenha menos de 201 elementos
    start = max(center - 100, 0)
    end = min(center + 100, len(msg.ranges) - 1)
    laser_readings = list(msg.ranges[start:end+1])
def navigation_controller():
    global position, orientation, waypoints
    cmd = Twist()
    if position[0] is None or position[1] is None or orientation is None:
        # Não temos posição/orientação ainda
        return
    if not waypoints:
        # Parar o robô quando todos os alvos forem alcançados
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        cmd_publisher.publish(cmd)
        return
    target_position = waypoints[0]
    dx = target_position[0] - position[0]
    dy = target_position[1] - position[1]
    distance_to_target = math.sqrt(dx**2 + dy**2)
    desired_angle = math.atan2(dy, dx)
    angle_error = desired_angle - orientation
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize entre -pi e pi
    if distance_to_target < 0.3:
        waypoints.pop(0)  # Alvo atingido
    elif detect_obstacle():
        # Evitar obstáculo: recua e gira
        cmd.linear.x = -0.1
        cmd.angular.z = 1.5
    else:
        if abs(angle_error) > 0.1:
            # Gira para alinhar antes de andar
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0 * angle_error
        else:
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
    cmd_publisher.publish(cmd)
def main(args=None):
    global cmd_publisher, red_robot
    rclpy.init(args=args)
    red_robot = Node('differential_drive_navigator')
    cmd_publisher = red_robot.create_publisher(Twist, '/cmd_vel', 10)
    red_robot.create_subscription(Odometry, '/ground_truth', odometry_ground_callback, 10)
    red_robot.create_subscription(Odometry, '/odom', odometry_callback, 10)
    red_robot.create_subscription(LaserScan, '/base_scan', laser_scan_callback, 10)
    red_robot.create_timer(0.1, navigation_controller)
    rclpy.spin(red_robot)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    
___________________________________________________________________________________________________________________
Para esta tarefa foi feita a instalação do Ros2 e das váriaveis do ambiente, criação do colcon e workspace, criação
do pacote de trabalho substituindo os arquivos world e launch, criação do nodo em python e orientações do desafio Stage 
disponibilizadas no Playlist de Aulas de ROS 2 na plataforma AVA, na disciplina de introdução a Robótica.
https://www.youtube.com/playlist?list=PLhxZVyws6Ytssb_CJA5cKxY5IxecOvJao
____________________________________________________________________________________________________________________







"""
No ROS2 que le os waypoints do contour.json e faz a tartaruga
percorrer cada contorno usando controle proporcional.

Para cada waypoint alvo:
  rho   = distancia ate o alvo
  alpha = diferenca entre angulo desejado e angulo atual
  v_lin = Kp_lin * rho      (so avanca se o erro angular for pequeno)
  v_ang = Kp_ang * alpha

Quando termina um contorno, levanta a caneta, vai ate o inicio do
proximo contorno, abaixa a caneta e continua.
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


def normalizar(a):
    """Mantem angulo em (-pi, pi].

    Truque: atan2(sin(a), cos(a)) tira a periodicidade e devolve sempre o
    angulo equivalente no intervalo principal. Sem isso, a tartaruga
    ocasionalmente decide girar o caminho longo (>180 graus).
    """
    return math.atan2(math.sin(a), math.cos(a))


class TurtleDraw(Node):
    KP_LIN = 1.5
    KP_ANG = 6.0
    LIMIAR_GIRO = 0.25   # se erro angular for maior, gira sem avancar
    TOL_POS = 0.08       # distancia para considerar que chegou no ponto

    def __init__(self):
        super().__init__('turtle_draw')

        self.declare_parameter('arquivo_contorno', 'output/contour.json')
        arq = os.path.abspath(self.get_parameter('arquivo_contorno').value)
        with open(arq) as f:
            self.tracos = json.load(f)['tracos']
        self.get_logger().info(
            f'{len(self.tracos)} contornos, '
            f'{sum(len(t) for t in self.tracos)} waypoints')

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.cb_pose, 10)
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')

        self.pose = None
        self.idx_traco = 0
        self.idx_wp = 0
        self.estado = 'WAIT'    # WAIT -> TRAVEL -> DRAW -> DONE
        # rclpy: a Future do call_async precisa ter referencia viva ate
        # completar, senao o garbage collector descarta a requisicao
        self._pendentes = []
        self.create_timer(0.05, self.controle)

    def cb_pose(self, msg):
        self.pose = msg

    def set_pen(self, abaixada):
        """Levanta ou abaixa a caneta. Nao espera resposta."""
        if not self.cli_pen.service_is_ready():
            return
        req = SetPen.Request()
        req.r, req.g, req.b = 30, 30, 30
        req.width = 3
        req.off = 0 if abaixada else 1
        self._pendentes.append(self.cli_pen.call_async(req))

    def parar(self):
        self.pub.publish(Twist())

    def controle(self):
        if self.pose is None:
            return

        if self.estado == 'WAIT':
            self.set_pen(False)
            self.estado = 'TRAVEL'
            return

        if self.estado == 'DONE':
            self.parar()
            return

        alvo_x, alvo_y = self.tracos[self.idx_traco][self.idx_wp]
        dx = alvo_x - self.pose.x
        dy = alvo_y - self.pose.y
        rho = math.hypot(dx, dy)

        if rho < self.TOL_POS:
            if self.estado == 'TRAVEL':
                # Chegamos no inicio do segmento: abaixa caneta e desenha
                self.set_pen(True)
                self.estado = 'DRAW'
                self.idx_wp = 1
            else:
                self.idx_wp += 1
                if self.idx_wp >= len(self.tracos[self.idx_traco]):
                    # Fim do segmento: vai para o proximo
                    self.idx_traco += 1
                    self.idx_wp = 0
                    if self.idx_traco >= len(self.tracos):
                        self.estado = 'DONE'
                        self.parar()
                        self.get_logger().info('Desenho concluido')
                        return
                    self.set_pen(False)
                    self.estado = 'TRAVEL'
            return

        # Controle proporcional (go-to-goal classico):
        #   alpha = diferenca entre o angulo desejado e o angulo atual
        #   v_lin proporcional a distancia (rho), v_ang proporcional a alpha
        alpha = normalizar(math.atan2(dy, dx) - self.pose.theta)
        # Se o erro angular for grande, gira primeiro sem avancar.
        # Caso contrario a tartaruga desenharia arcos no carry-return
        # entre o fim de uma linha e o inicio da proxima.
        v_lin = 0.0 if abs(alpha) > self.LIMIAR_GIRO else self.KP_LIN * rho
        v_ang = self.KP_ANG * alpha

        # Satura velocidades para evitar comandos absurdos no turtlesim
        cmd = Twist()
        cmd.linear.x = max(-2.0, min(2.0, v_lin))
        cmd.angular.z = max(-4.0, min(4.0, v_ang))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDraw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.parar()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

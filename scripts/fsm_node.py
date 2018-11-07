#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from transitions import Machine
from gpiozero import Servo
from topic_tools.srv import *
import math

class FSM(object):
    fsm_states = ['start',
            'inicio', 
            'media_vuelta_inter_1', 
            'buscar_cubo', 
            'avanzar', 
            'avanzar_cubo', 
            'encerrar_cubo', 
            'girar_linea', 
            'buscar_linea', 
            'seguir_linea_1_f', 
            'media_vuelta_inicio', 
            'seguir_linea_1_b', 
            'seguir_linea_2_f', 
            'girar_color_f', 
            'seguir_linea_3_f', 
            'soltar_cubo', 
            'media_vuelta_fin', 
            'seguir_linea_3_b', 
            'girar_color_b', 
            'seguir_linea_2_b']

    color = 'r'
    side_detected = 0
    orientation = 0.0
    action = 'null'
    cmd_vel_select = None
    vel = Twist()
    ready = False

    def stop_robot(self):
        self.cmd_vel_select('raw_cmd_vel')
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        cmd_vel_pub.publish(self.vel)

    def negro(self):
        return (self.color == 'k')

    def activar_seguidor_linea(self):
        self.action = 'following'
        print ("siguiendo linea")
        self.cmd_vel_select('follow_line_cmd_vel')
        if self.state == 'avanzar':
            rospy.sleep(0.5)
            self.timeout()
    
    def girar(self):
        self.action = 'turning'
        self.cmd_vel_select('angle_cmd_vel')
        if (self.state == 'media_vuelta_inter_1') or (self.state == 'media_vuelta_inicio') or (self.state == 'media_vuelta_fin'):
            print ('girando 180 grados')
            self.orientation = math.pi - 0.1
            angle_pub.publish(self.orientation)
        elif (self.state == 'girar_linea'):
            if self.negro():
                if self.side_detected == 1:
                    self.orientation = 105 * math.pi / 180
                else:
                    self.orientation = -105 * math.pi / 180
                angle_pub.publish(self.orientation)
                print ('girando 105 grados')
            else:
                if self.side_detected == 1:
                    self.orientation = 75 * math.pi / 180
                else:
                    self.orientation = -75 * math.pi / 180
                angle_pub.publish(self.orientation)
                print ('girando 75 grados')
        elif self.state == 'girar_color_f':
            if self.color == 'r':
                self.orientation = math.pi / 2
                angle_pub.publish(self.orientation)
                print ('girar rojo')
            elif self.color == 'g':
                self.orientation = -math.pi / 2
                angle_pub.publish(self.orientation)
                print ('girar verde')
            else:
                print ('girar azul')
        elif self.state == 'girar_color_b':
            if self.color == 'r':
                self.orientation = -math.pi / 2
                angle_pub.publish(self.orientation)
                print ('girar rojo b')
            elif self.color == 'g':
                self.orientation = math.pi / 2
                angle_pub.publish(self.orientation)
                print ('girar verde b')
            else:
                print ('girar azul b')

    def sondeo(self):
        self.action = 'null'
        print ('sondeando')
        self.cmd_vel_select('angle_cmd_vel')
        self.orientation = -math.pi / 2
        angle_pub.publish(self.orientation)
        while not self.ready:
            pass
        self.ready = False
        self.action = 'searching'
        found = False
        for i in range(10):
            self.orientation = math.pi / 10
            angle_pub.publish(self.orientation)
            while not self.ready:
                pass
            self.ready = False
            if self.state != 'buscar_cubo':
                found = True
                break
        if not found:
            self.action = 'null'
            self.orientation = math.pi / 2
            angle_pub.publish(self.orientation)
            while not self.ready:
                pass
            self.ready = False
            self.no_cubo()
        
    def perseguir_cubo(self):
        self.action = 'hunting'
        print ('persiguiendo cubo')
        self.cmd_vel_select('follow_color_cmd_vel')

    def cerrar_brazo(self):
        self.action = 'hugging'
        print ('cerrando brazo')
        servo.min()
        rospy.sleep(1)
        self.cubo_encerrado()

    def encontrar_linea(self):
        self.action = 'returning'
        print ('buscando linea')
        self.cmd_vel_select('raw_cmd_vel')
        self.vel.linear.x = 0.2
        self.vel.angular.z = 0.0
        cmd_vel_pub.publish(self.vel)

    def abrir_brazo(self):
        self.action = 'realising'
        print ('abriendo_brazo')
        servo.max()
        rospy.sleep(1)
        self.cubo_liberado()
    
    def __init__(self):
        self.machine = Machine(model=self, states=FSM.fsm_states, initial='start')
        self.machine.add_transition('comenzar', 'start', 'inicio', after='activar_seguidor_linea')
        self.machine.add_transition('interseccion', 'inicio', 'media_vuelta_inter_1', after='girar')
        self.machine.add_transition('giro_completo', 'media_vuelta_inter_1', 'buscar_cubo', after='sondeo')        
        self.machine.add_transition('no_cubo', 'buscar_cubo', 'avanzar', after='activar_seguidor_linea')
        self.machine.add_transition('timeout', 'avanzar', 'buscar_cubo', after='sondeo')
        self.machine.add_transition('interseccion', 'avanzar', 'media_vuelta_inicio', after='girar')
        self.machine.add_transition('cubo', 'buscar_cubo', 'avanzar_cubo', after='perseguir_cubo')
        self.machine.add_transition('cubo_atrapado', 'avanzar_cubo', 'encerrar_cubo', after='cerrar_brazo')
        self.machine.add_transition('cubo_encerrado', 'encerrar_cubo', 'girar_linea', after='girar')
        self.machine.add_transition('giro_completo', 'girar_linea', 'buscar_linea', after='encontrar_linea')
        self.machine.add_transition('linea', 'buscar_linea', 'seguir_linea_1_f', conditions='negro', after='activar_seguidor_linea')
        self.machine.add_transition('interseccion', 'seguir_linea_1_f', 'media_vuelta_inicio', after='girar')
        self.machine.add_transition('giro_completo', 'media_vuelta_inicio', 'inicio', after='activar_seguidor_linea')
        self.machine.add_transition('linea', 'buscar_linea', 'seguir_linea_1_b', unless='negro', after='activar_seguidor_linea')
        self.machine.add_transition('interseccion', 'seguir_linea_1_b', 'seguir_linea_2_f', after='activar_seguidor_linea')
        self.machine.add_transition('interseccion', 'seguir_linea_2_f', 'girar_color_f', after='girar')
        self.machine.add_transition('giro_completo', 'girar_color_f', 'seguir_linea_3_f', after='activar_seguidor_linea')
        self.machine.add_transition('interseccion', 'seguir_linea_3_f', 'soltar_cubo', after='abrir_brazo')
        self.machine.add_transition('cubo_liberado', 'soltar_cubo', 'media_vuelta_fin', after='girar')
        self.machine.add_transition('giro_completo', 'media_vuelta_fin', 'seguir_linea_3_b', after='activar_seguidor_linea')
        self.machine.add_transition('interseccion', 'seguir_linea_3_b', 'girar_color_b', after='girar')
        self.machine.add_transition('giro_completo', 'girar_color_b', 'seguir_linea_2_b', after='activar_seguidor_linea')
        self.machine.add_transition('interseccion', 'seguir_linea_2_b', 'buscar_cubo', after='sondeo')


def turn_completed(data):
    if fsm.action == 'turning':
        fsm.stop_robot()
        fsm.giro_completo()
    if fsm.action == 'searching':
        fsm.ready = True

def cube_detected(data):
    if fsm.action == 'searching':
        fsm.stop_robot()
        if data.data:
            fsm.cubo()
        else:
            fsm.no_cubo()

def cube_catched(data):
    if fsm.action == 'hunting':
        fsm.stop_robot()
        fsm.color = data.data
        fsm.cubo_atrapado()

def line(data):
    if fsm.action == 'returning':
        fsm.stop_robot()
        fsm.linea()

def cross(data):
    if fsm.action == 'following':
        fsm.stop_robot()
        fsm.interseccion()

fsm = FSM()
servo = Servo(17)

if __name__ == '__main__':
    rospy.wait_for_service('mux_cmd_vel/select')
    try:
        rospy.init_node('fsm_node', anonymous=True)
        rospy.Subscriber("turn_complete", Empty, turn_completed)
        rospy.Subscriber("cube_detectec", Bool, cube_detected)
        rospy.Subscriber("cube_color", String, cube_catched)
        rospy.Subscriber("line_detected", Empty, line)
        rospy.Subscriber("line_cross", Empty, cross)
        angle_pub = rospy.Publisher('angle', Float32, queue_size=10)
        cmd_vel_pub = rospy.Publisher('raw_cmd_vel', Twist, queue_size=10)
        fsm.cmd_vel_select = rospy.ServiceProxy('mux_cmd_vel/select', MuxSelect)
        fsm.stop_robot()

        servo.max()
        fsm.comenzar()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

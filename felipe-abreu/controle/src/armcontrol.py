#!/usr/bin/env python

# (C) CESAR 2014 
# BSD license


# 0xFF 0x08 0x00 0x08 0x00 0x08 0x00 0x08 0x00 0x02 0x00 0x01 0x00 0x7D 0x00 0x00 0x5F

# ff 8 0 8 0 8 0 8 0 2 0 1 0 80 0 0 5c --> backhoe standard position package

import rospy
import serial
import struct

import time

from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Joints are defined as follows (from the urdf file):
#
# j0 = Joint base_arm_to_base2
# j1 = Joint base_to_shoulder
# j2 = Joint shoulder_to_elbow
# j3 = Joint elbow_to_wrist
# j4 = Joint wrist_to_grip_base
# j5 = Joint grip_base_to_right_finger        [axis = +y]
# j6 = Joint grip_base_to_left_finger (j6=j5) [axis = -y]
#
# delta: delta*16 == time, in ms, for executing a single movement

def packshort(indata): # Return a string containing the value 'indata' in the format '>h' (big-endian, 2 bytes -> '\xFFFF')
    #serialize a short BE
    return struct.pack('>h',indata) 


def prep_msg(j0, j1, j2, j3, j4, j5, delta, init): # converte os valores de junta de rad para uma medida legivel e comprime todos numa so string

    # Aqui o msg vai virar uma string de 18 caracteres ASCII em que o primeiro eh o equivalente a \xFF na tabela, o segundo eh equivalente ao j0 jah convertido de radianos para a medida reconhecida pelo arbotix, ..., o 16 eh o equivalente ao delta, o 17 e o 18 sao \x00 
    # Quando init ==1, 
   
    if (init == 0):

	msg = '\xFF' + packshort(j0 / 2.70526 * 2048. + 2048.) \
        + packshort(j1 / 1.57080 * 1024. + 2048.) \
        + packshort(j2 / 1.57080 * 1024. + 2048.) \
        + packshort(j3 / 1.57080 * 1024. + 2048.) \
        + packshort(j4 / 2.70526 * 512. + 512.) \
        + packshort(j5 / 0.01200 * 512. + 512.) \
        + chr(delta) \
        + '\x00' + '\x00'

    else:
	msg = '\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40'


    tot = 0
    # verificar que soh eh ate 15 (apesar de o primeiro resultado de csum ser 255-64 = 191 e esse valor estar certo)
    for i in xrange(1, 16):
        tot += ord(msg[i])

    tot = tot % 256

    csum = 255 - tot

    rospy.loginfo("tot is %i" % csum)

    msg = msg + chr(csum)

    rospy.loginfo("msg is %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x" % (ord(msg[0]),ord(msg[1]),ord(msg[2]),ord(msg[3])
                                          ,ord(msg[4])
                                          ,ord(msg[5])
                                          ,ord(msg[6])
                                          ,ord(msg[7])
                                          ,ord(msg[8])
                                          ,ord(msg[9])
                                          ,ord(msg[10])
                                          ,ord(msg[11])
                                          ,ord(msg[12])
                                          ,ord(msg[13])
                                          ,ord(msg[14])
                                          ,ord(msg[15])
                                          ,ord(msg[16])
                                      ))
    return msg



def control_callback(jsm):  # Atribui os valores de juntas recebidos por uma mensagem do /joint_state as variaveis j0, ..., j5 [entender count]
    global count

    if (count < 20):
	count = count + 1
	return

    count = 0

    rospy.loginfo("got arm ctrl msg")

    delta = 128

    for i in xrange(0, len(jsm.name)):

	if (jsm.name[i] == "base_arm_to_base2"):
    		j0 = jsm.position[i]

	elif (jsm.name[i] == "base_to_shoulder"):
		j1 = jsm.position[i]

        elif (jsm.name[i] == "shoulder_to_elbow"):
		j2 = jsm.position[i]

        elif (jsm.name[i] == "elbow_to_wrist"):
                j3 = jsm.position[i]

        elif (jsm.name[i] == "wrist_to_grip_base"):
                j4 = jsm.position[i]

        elif (jsm.name[i] == "grip_base_to_right_finger"):
                j5 = jsm.position[i]

  
    msg = prep_msg(j0, j1, j2, j3, j4, j5, delta, 0)

    bsent = ser.write(msg)
    ser.flush()

    rospy.loginfo("send arm ctrl msg of %d bytes" % bsent)


if __name__=="__main__":
    global ser, count

    rospy.init_node("armcontrol")     # inicia o noh armcontrol
    rospy.loginfo("ROS arm control")  # so para ter certeza

    port_name = rospy.get_param('~port','/dev/ttyUSB0')               # a variavel port_name recebe o nome da porta usb que liga o pc ao robo
    baud = int(rospy.get_param('~baud','38400'))                      # a variavel baud recebe a frequencia de transmissao da conexao (38400) em int
    rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )  # aviso de inicio de conexao
    
    count = 0

    r = rospy.Rate(100)                     # 100hz = frequencia com a qual o loop que acaba em r.sleep ocorre (linha 171) [ainda precisa?]

    ser = serial.Serial(port_name, baud)    # A conexao eh estabelecida, de fato, soh agora por meio da variavel ser

    time.sleep(12)  # O microcontrolador soh comeca a receber pacotes apos um periodo de 10s apos a conexao ser estabalecida, entao esperamos 12s antes de enviar o primeiro pacote

    msg = prep_msg(0, 0, 0, 0, 0, 0, 0, 1)  # Preparando o primeiro pacote, que vai apenas indicar a pose central do backhoe
    rospy.loginfo("send arm ctrl msg")      # Aviso de confirmacao
    ser.write(msg)                          # Envio do primeiro pacote

    ser.flush()      # Nao sei se ainda eh necessario
    ser.read(5)      # Tambem nao sei se eh necessario

    # Chama o callback passando as publicacoes de /joint_state (tipo JointState) como parametro para a funcao. control_callback eh invocada sempre que uma mensagem do tipo JointState eh publicada 

    rospy.Subscriber("/joint_states", JointState, control_callback)   # (topico, tipo, funcao) 

    while not rospy.is_shutdown():
        r.sleep()

    ser.close()


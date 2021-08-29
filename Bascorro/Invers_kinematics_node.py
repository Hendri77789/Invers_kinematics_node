#!/usr/bin/env python

# import library / package yang digunakan
import rospy
import math
import numpy as np
from std_msgs.msg import UInt16
from dynamixel_control.msg import DynamixelPos, DynamixelPosList
from inverse_kinematics.msg import IKTarget, FullBodyIK, offset

# deklarasi dan defini variabel yang akan digunakan
l1 = 13.9
l2 = 14.6
offsetFront = 12
offsetSide = 5
cam_pitch = 180

# Mendefinisikan pengirim dengan nama "set_position" dengan mengirimkan data yang berada pada "DynamixelPosList" dengan ukuran queue 10 hz
dynamixelPosList_pub = rospy.Publisher('set_position', DynamixelPosList, queue_size=10)

# Mendefinisikan pergerakan kaki kanan
def r_leg_ik(koordinatX, koordinatY,  koordinatZ, rotate):
    x = koordinatX
    y = koordinatZ
    z = koordinatY

# Mencoba pergerakan
    try:
        A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
        B = math.sqrt(1-(A**2))
        tetha3 = math.degrees(math.acos(A))
        
        buff1 = l2*math.cos(math.radians(tetha3)) + l1
        buff2 = math.sqrt(x**2 + y**2)
        buff3 = l2*math.sin(math.radians(tetha3))

        # Mendefinisikan variabel
        C = math.degrees(math.atan2(buff2,z))
        D = math.degrees(math.atan2(buff3,buff1))
        tetha2 = C - D
        tetha5 = math.degrees(math.atan2(x,y))
        tetha1 = tetha5
        tetha4 = tetha3 + tetha2 - 90
        tetha6 = rotate

        # Pengembalian nilai dari variabel di atas
        return [tetha1+180+offsetSide, 90-offsetFront+tetha2, -tetha3+180, tetha4+180, tetha5+180+offsetSide, tetha6+180]

    # Perintah jika nilai error
    except ValueError:
        rospy.logerr('[IK_Target] Value Error')
        return [180, 180, 180, 180, 180, 180]   # Pengembalian nilai
    
# Mendefinisikan pergerakan kaki kiri
def l_leg_ik(koordinatX, koordinatY,  koordinatZ, rotate):
    x = koordinatX
    y = koordinatZ
    z = koordinatY

    # Mencoba pergerakan
    try:
        A = (x**2 + y**2 + z**2 - l1**2 - l2**2) /(2*l1*l2)
        B = math.sqrt(1-(A**2))
        tetha3 = math.degrees(math.acos(A))
        
        buff1 = l2*math.cos(math.radians(tetha3)) + l1
        buff2 = math.sqrt(x**2 + y**2)
        buff3 = l2*math.sin(math.radians(tetha3))

        # Mendefisikan variabel
        C = math.degrees(math.atan2(buff2,z))
        D = math.degrees(math.atan2(buff3,buff1))
        tetha2 = C - D
        tetha5 = math.degrees(math.atan2(x,y))
        tetha1 = tetha5
        tetha4 = tetha3 + tetha2 - 90
        tetha6 = rotate

        # Pengembalian nilai dari variabel di atas
        return [-tetha1+180-offsetSide, 270+offsetFront-tetha2, tetha3+180, -tetha4+180, -tetha5+180-offsetSide, -tetha6+180]

    # Perintah jika nilai error
    except ValueError:
        rospy.logerr('[IK_Target] Value Error')
        return [180, 180, 180, 180, 180, 180]   # Pengembalian nilai

# Mendefisikan fungsi r_leg_ik_handler ()
def r_leg_ik_handler(data):
    result = r_leg_ik(data.x, data.y, data.z, data.rotate)
    id = [9, 11, 13, 15, 17, 7]

    # Mendefisikan variabel dataPub dengan DynamixelPosList()
    dataPub = DynamixelPosList()

    # Fungsi for
    for idx in range(6):
        dynamixelPos = DynamixelPos()
        dynamixelPos.id = id[idx]
        dynamixelPos.position = result[idx]
        dataPub.dynamixel.append(dynamixelPos)

    # Mempublish variabel dynamixelPosList_pub
    dynamixelPosList_pub.publish(dataPub)

# Mendefisikan fungsi l_leg_ik_handler ()
def l_leg_ik_handler(data):
    result = l_leg_ik(data.x, data.y, data.z, data.rotate)
    id = [10, 12, 14, 16, 18, 8]

    # Mendefisikan variabel dataPub dengan DynamixelPosList()
    dataPub = DynamixelPosList()

    # Fungsi for
    for idx in range(6):
        dynamixelPos = DynamixelPos()
        dynamixelPos.id = id[idx]
        dynamixelPos.position = result[idx]
        dataPub.dynamixel.append(dynamixelPos)

    # Mempublish variabel dynamixelPosList_pub
    dynamixelPosList_pub.publish(dataPub)

# Mendefisikan fungsi full_body_ik_handler()
def full_body_ik_handler(data):
    right_leg_ik = r_leg_ik(data.right_leg.x, data.right_leg.y, data.right_leg.z, data.right_leg.rotate)
    left_leg_ik = l_leg_ik(data.left_leg.x, data.left_leg.y, data.left_leg.z, data.left_leg.rotate)
    right_leg_id = [9, 11, 13, 15, 17, 7]
    left_leg_id = [10, 12, 14, 16, 18, 8]

    # Mendefisikan variabel dataPub dengan DynamixelPosList()
    dataPub = DynamixelPosList()
    # dynamixelPos = DynamixelPos()
    # dynamixelPos.id = 20
    # dynamixelPos.position = cam_pitch
    # dataPub.dynamixel.append(dynamixelPos)

    # Fugsi for
    for idx in range(6):
        dynamixelPos = DynamixelPos()
        dynamixelPos.id = right_leg_id[idx]
        dynamixelPos.position = right_leg_ik[idx]
        dataPub.dynamixel.append(dynamixelPos)
    for idx in range(6):
        dynamixelPos = DynamixelPos()
        dynamixelPos.id = left_leg_id[idx]
        dynamixelPos.position = left_leg_ik[idx]
        dataPub.dynamixel.append(dynamixelPos)

    # Mempublish variabel dynamixelPosList_pub
    dynamixelPosList_pub.publish(dataPub)
    
# Mendefinisikan fungsi setCamPitchHandler ()
def setCamPitchHandler(data):
    global cam_pitch
    cam_pitch = data.data

# Mendefinisikan fungsi setCamPitchHandler ()
def setOffsetHandler(data):
    global offsetSide, offsetFront
    offsetFront = data.front_offset
    offsetSide = data.side_offset

# fungsi if
if __name__ == '__main__' :
    rospy.init_node('inverse_kinematics_node')

    rospy.Subscriber('r_leg_target', IKTarget, r_leg_ik_handler)    # Menerima data dari node "r_leg_target" yang berisi IKTarget yang akan diolah dengan fungsi r_leg_ik_handler
    rospy.Subscriber('l_leg_target', IKTarget, l_leg_ik_handler)    # Menerima data dari node "l_leg_target" yang berisi IKTarget yang akan diolah dengan fungsi l_leg_ik_handler
    rospy.Subscriber('full_body_target', FullBodyIK, full_body_ik_handler)  # Menerima data dari node "full_body_target" yang berisi FullBodyIK yang akan diolah dengan fungsi full_body_ik_handler
    rospy.Subscriber('set_camPitch', UInt16, setCamPitchHandler)    # Menerima data dari node "set_camPitch" yang berisi UInt16 yang akan diolah dengan fungsi setCamPitchHandler
    rospy.Subscriber('set_offset', offset, setOffsetHandler)    # Menerima data dari node "set_offset" yang berisi offset yang akan diolah dengan fungsi setOffsetHandler

    rate = rospy.Rate(10)
    rospy.spin()
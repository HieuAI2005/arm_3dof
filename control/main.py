from forward import inverse_kinematics
from send_mess import send_message
#from ik_haianh import inverse_kinematics
import time

'''
BBox 1:
t1, t2, t3 = 45, -10, 60
# BBox 2: 
# t1, t2, t3 = -35, -30, 90
Base:
t1, t2, t3 = 0, 0, 60
'''

while True:
    x = float(input("Enter x: "))
    y = float(input("Enter y: "))
    place = float(input("Enter place: "))
    sol = inverse_kinematics(x, y)
    print(sol)
    current_angle = send_message((sol))
    time.sleep(1)
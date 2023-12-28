
#Updating Raspberry Pi
sudo apt-get install rpi-updates
sudo rpi-update
sudo apt-get update
sudo apt-get upgrade
#Enabling Camera
sudo raspi-config
enable camera
finish
#PyCharm IDE
#Importing Libraries
import time
import RPi.GPIO as GPIO
import cv2
import math
import numpy as np
#Declairing Variables
channel = 12
x1 = 0
y1 = 0
x = 0
y = 0
i = 0
#Capturing Video From Camera
cap= cv2.VideoCapture(0)
#Function of Inverse Kinematics
def inverse_kinamtics(x, y, z, phi):
distance = math.sqrt(x ** 2 + y ** 2 + (z - 5.1) ** 2)
print("distance is ==", distance)
if distance <= 34.2:
theta1 = math.atan2(y, x)
# print(theta1)
theta1_degree = theta1 * 57.2957795
print("theta1=", theta1_degree)
#########################Theta2################
dami1 = (z - 8.1)
dami2 = (x * math.cos(theta1) + y * math.sin(theta1))
dami3 = (dami1 ** 2 + dami2 ** 2) / 34.2
theta2 = math.atan2(dami3, -math.sqrt(dami1 ** 2 + dami2 ** 2
- dami3 ** 2)) - math.atan2(dami2, dami1)
theta2_degree = theta2 * 57.2957795 + 7
print("theta2=", theta2_degree)
#########################Theta3################
dami4 = 17.1
dami5 = 17.1
dami6 = ((math.cos(theta2) * (dami1 + dami2)) +
(math.sin(theta2) * (dami1 - dami2)) - (17.1))
theta3 = math.atan2(dami6, math.sqrt(dami4 ** 2 + dami5 ** 2 -
dami6 ** 2)) - math.atan2(dami5, dami4)
# print(theta3)
theta3_degree = theta3 * 57.2957795
print("theta3=", theta3_degree)
#########################Theta4################
theta4 = (phi - theta2 - theta3)
# print(theta4)
theta4_degree = theta4 * 57.2957795
print("theta4", theta4_degree)
return (theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
else:
print("end factor is out of reach")
#Function of Mapping Angles
def maprange(a, b, s):
(a1, a2), (b1, b2) = a, b
return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))
def Map_fun(theta1_degree, theta2_degree, theta3_degree, theta4_degree):
#########this rounding is for third quardraten##################
if x < 0 and y < 0:
angle1 = -round(theta1_degree, 2) + 60 + 90
else:
angle1 = round(theta1_degree, 2) + 60
############rounding and offset of motor hardware
angle2 = round(theta2_degree, 2) + 60
angle3 = round(theta3_degree, 2) + 90 + 60
angle4 = round(theta4_degree, 2) + 60
#################################################
###################### Mapping function###############
####this for to map the joint angle with motor angle########
angle1_map = maprange((0, 300), (-150, 150), angle1)
print(angle1_map)
angle2_map = maprange((0, 300), (-150, 150), angle2)
print(angle2_map)
angle3_map = maprange((0, 300), (-150, 150), angle3)
print(angle3_map)
angle4_map = maprange((0, 300), (-150, 150), angle4)
print(angle4_map)
# print("%2g maps to %g" % (s, maprange((0, 300), (-150, 150), s)))
# Connect to the serial port
from pyax12.connection import Connection
serial_connection = Connection(port="/dev/ttyUSB0", baudrate=1000000)
###########ID of dynamixel motors###############
dynamixel_id1 = 8
dynamixel_id2 = 4
dynamixel_id3 = 5
dynamixel_id4 = 6
# voltage = Connection.get_present_voltage(dynamixel_id=dynamixel_id1)
#######Method to move motors to desired angle with desired
speed###########
serial_connection.goto(dynamixel_id1, angle1_map, speed=80,
degrees=True)
time.sleep(0.005) # Wait 1 second
serial_connection.goto(dynamixel_id2, angle2_map, speed=80,
degrees=True)
time.sleep(0.005) # Wait 1 second
serial_connection.goto(dynamixel_id3, angle3_map, speed=80,
degrees=True)
time.sleep(0.005) # Wait 1 second
serial_connection.goto(dynamixel_id4, angle4_map, speed=80,
degrees=True)
# Close the serial connection
serial_connection.close()
# y = y + 0.36
# time.sleep(2)
# time.sleep(0.1) # Wait 1 second
#Function for Pneumatic Pump
def motor_on(pin):
GPIO.setmode(GPIO.BCM)
GPIO.setup(channel, GPIO.OUT)
GPIO.output(pin, GPIO.HIGH)
def motor_off(pin):
GPIO.setmode(GPIO.BCM)
GPIO.setup(channel, GPIO.OUT)
GPIO.output(pin, GPIO.LOW)
Defining the Intensity of Colors
while (1):
_, img = cap.read()
# converting frame(img i.e BGR) to HSV (hue-saturation-value)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# definig the range of red color
red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)
# defining the Range of green color
green_lower = np.array([50, 100, 100], np.uint8)
green_upper = np.array([70, 255, 255], np.uint8)
# finding the range of red and green color in the image
red = cv2.inRange(hsv, red_lower, red_upper)
green = cv2.inRange(hsv, green_lower, green_upper)
# Morphological transformation, Dilation
kernal = np.ones((5, 5), "uint8")
red = cv2.dilate(red, kernal)
res = cv2.bitwise_and(img, img, mask=red)
green = cv2.dilate(green, kernal)
res2 = cv2.bitwise_and(img, img, mask=green)
# Tracking the Red Color
(_, contours, hierarchy) = cv2.findContours(red, cv2.RETR_TREE,
cv2.CHAIN_APPROX_SIMPLE)
for pic, contour in enumerate(contours):
area = cv2.contourArea(contour)
if (area > 300):
x, y, w, h = cv2.boundingRect(contour)
img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0,
255), 2)
cv2.putText(img, "RED Ball", (x, y),
cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
Xaxis = round((x - 308) / 8.8, 4)
Yaxis = -round((y - 455) / 8.8, 4)
print("R_X =", Xaxis, "cm", "R_Y =", Yaxis, "cm")
x = y = 0
# Tracking the Green Color
(_, contours, hierarchy) = cv2.findContours(green, cv2.RETR_TREE,
cv2.CHAIN_APPROX_SIMPLE)
for pic, contour in enumerate(contours):
area = cv2.contourArea(contour)
if (area > 300):
x1, y1, w, h = cv2.boundingRect(contour)
img = cv2.rectangle(img, (x1, y1), (x1 + w, y1 + h), (0,
255, 0), 2)
cv2.putText(img, "Green Ball", (x1, y1),
cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0))
X1axis = round((x1 - 308) / 8.8, 4)
Y1axis = -round((y1 - 455) / 8.8, 4)
print("G_X =", X1axis, "cm", "G_Y =", Y1axis, "cm")
x1 = y1 = 0
Calling the Function to Activate the Manipulator
i = i + 1
try:
while (i > 5):
#####################For Red ########################
if (Xaxis > -35):
cv2.imshow("Color Tracking", img)
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics( (Xaxis+2),( Yaxis-2), 15, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(2)
motor_on(channel)
time.sleep(1)
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics((Xaxis+2),( Yaxis-2), 9, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(2)
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics((Xaxis+2), ( Yaxis-2), 20, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(2)
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics(-18, 0, 16, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(3)
motor_off(channel)
time.sleep(1)
elif(X2axis > -35):
cv2.imshow("Color Tracking", img)
##############for Green##############################
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics((X2axis+2),( Y2axis-2), 15, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(2)
motor_on(channel)
time.sleep(1)
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics((X2axis+2),( Y2axis-2), 9, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(2)
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics((X2axis+2), (Y2axis-2), 20, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(2)
theta1_degree, theta2_degree, theta3_degree,
theta4_degree = inverse_kinamtics(18, 0, 16, 0)
Map_fun(theta1_degree, theta2_degree, theta3_degree,
theta4_degree)
time.sleep(3)
motor_off(channel)
time.sleep(1)
else:
print("there is no ball")
i = 0
cv2.imshow("Color Tracking", img)
if cv2.waitKey(1) & 0xFF == ord('q'):
cap.release()
cv2.destroyAllWindows()
break
except TypeError:
print("error")

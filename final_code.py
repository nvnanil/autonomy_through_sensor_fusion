"""
Author: Naveen Anil
Date: May 3rd 2023
Description: This code adds the autonomous capability to the vehicle. The robot localizes itself and place the different colored blocks
in the respective portion of the arana
"""
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import serial
import math
import matplotlib.pyplot as plt
import imutils
import RPi.GPIO as gpio
import email
from email01 import *
    
def init(): #Pin initialization
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT) 
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT) 
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)
    gpio.setup(36,gpio.OUT)
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover(): #Stopping the motors
        gpio.output(31, False)
        gpio.output(33, False)
        gpio.output(35, False)
        gpio.output(37, False)
        gpio.cleanup()

def imu(): #Reading imu data
    global ser,count
    ser.reset_input_buffer()
    while (ser.in_waiting == 0):
        continue
        #Read serial stream
    line = ser.readline()
    line = line.rstrip().lstrip()
    line = str(line)
    line = line.strip("'")
    line = line.strip("b'")
    angle = float(line)
    angle = round(angle)
    angle = int(angle)
    return angle
    #print(line,"\n")
        
def ultrasonic(): #Reading values from the ultrasonic sensor
    global trig,echo    
    try:
        #Ensure output has no value
        gpio.output(trig,False)
        time.sleep(0.01)
        #Generate Pulse
        gpio.output(trig,True)
        time.sleep(0.00001)
        gpio.output(trig,False)
        #Generate echo time signal
        while gpio.input(echo) ==0 :
            pulse_start = time.time()
        while gpio.input(echo) == 1:
            pulse_end  = time.time()

        pulse_duration = pulse_end - pulse_start
        # Convert time to distance
        distance = pulse_duration* 17150
        distance =  round(distance,2)
        return distance,True
    except :
        pass
    return -1, False

def ultrasonic_reading():
    distance_list_flag = True
    distance_list = []
    while (distance_list_flag):
        for k in range(3):
            distance_i , flag = ultrasonic()
            if(flag == True):
                distance_list.append(distance_i)
                distance_list_flag = False
    average_distance = sum(distance_list)/len(distance_list)
    distance_m = average_distance/100
    return (distance_m)

def forward(): #Controlling forward speed
    global counterBR, counterFL, pwm_right, pwm_left,mov_speed,prev_error, kd, ki, t_error
    diff_ticks = counterFL - counterBR
    gain = (kp * abs(diff_ticks))
    t_error += diff_ticks
    prev_error = diff_ticks
    fnl_speed = mov_speed + gain
    fnl_speed1 = mov_speed - gain
    if(fnl_speed <0 or fnl_speed > 100):
        fnl_speed = mov_speed
    if(fnl_speed1 <0 or fnl_speed1 > 100):
        fnl_speed1 = mov_speed
    if(diff_ticks < 0):
        pwm_right.ChangeDutyCycle(fnl_speed)
        pwm_left.ChangeDutyCycle(fnl_speed1)
    elif(diff_ticks > 0):
        pwm_right.ChangeDutyCycle(fnl_speed1)
        pwm_left.ChangeDutyCycle(fnl_speed)
    else:
        pwm_right.ChangeDutyCycle(mov_speed)
        pwm_left.ChangeDutyCycle(mov_speed)

def reverse(): #Controlling reverse speed
    global counterBR, counterFL, pwm_left_b, pwm_right_b,mov_speed,prev_error, kd, ki, t_error
    diff_ticks = counterFL - counterBR
    gain = (kp * abs(diff_ticks))
    t_error += diff_ticks
    prev_error = diff_ticks
    fnl_speed = mov_speed + gain
    fnl_speed1 = mov_speed - gain
    if(fnl_speed <0 or fnl_speed > 100):
        fnl_speed = mov_speed
    if(fnl_speed1 <0 or fnl_speed1 > 100):
        fnl_speed1 = mov_speed
    if(diff_ticks < 0):
        pwm_left_b.ChangeDutyCycle(fnl_speed)
        pwm_right_b.ChangeDutyCycle(fnl_speed1)
    elif(diff_ticks > 0):
        pwm_left_b.ChangeDutyCycle(fnl_speed1)
        pwm_right_b.ChangeDutyCycle(fnl_speed)
    else:
        pwm_left_b.ChangeDutyCycle(mov_speed)
        pwm_right_b.ChangeDutyCycle(mov_speed)

def dist_ticks(d):
    wheel_rev = ((1/(np.pi*0.065)) * d)
    dis_ticks = 120*8* wheel_rev
    return int(dis_ticks)

def deg_ticks(deg):
    wheel_rev = (1/(np.pi*0.065)) * (((2*np.pi*85*1.4)/(1000*360))*deg)
    a_ticks = 120*8* wheel_rev
    return int(a_ticks) 
        
def turn_right(): #Controlling turning
    global counterBR, counterFL, pwm_right, pwm_right_b,speed, kp,kd,t_error,prev_error
    diff_ticks = counterFL - counterBR
    gain = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error))
    t_error += diff_ticks
    prev_error = diff_ticks
    f_speed = speed + gain
    if(f_speed < 0 or f_speed > 100):
        f_speed = speed
    if(diff_ticks < 0):
        pwm_right.ChangeDutyCycle(f_speed)
        pwm_right_b.ChangeDutyCycle(speed)
    elif(diff_ticks > 0):
        pwm_right.ChangeDutyCycle(speed)
        pwm_right_b.ChangeDutyCycle(f_speed)
    else:
        pwm_right.ChangeDutyCycle(speed)
        pwm_right_b.ChangeDutyCycle(speed)

def turn_left():
    global counterBR, counterFL, pwm_left_b, pwm_left, speed, kp, kd, t_error, prev_error
    diff_ticks = counterFL - counterBR
    gain = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error))
    t_error += diff_ticks
    prev_error = diff_ticks
    f_speed = speed + gain
    
    if(f_speed < 0 or f_speed > 100):
        f_speed = speed
    if(diff_ticks < 0):
        pwm_left_b.ChangeDutyCycle(f_speed)
        pwm_left.ChangeDutyCycle(speed)
    elif(diff_ticks > 0):
        pwm_left_b.ChangeDutyCycle(speed)
        pwm_left.ChangeDutyCycle(f_speed)
    else:
        pwm_left_b.ChangeDutyCycle(speed)
        pwm_left.ChangeDutyCycle(speed)
    
def tforward_ticks(d_ticks):
    global counterBR,counterFL,pwm_right,pwm_left,buttonFL,buttonBR
    counterBR = 0
    counterFL = 0
    i=0
    while (counterBR < d_ticks or counterFL < d_ticks):
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12)) #Defining right back encoder
            counterBR += 1

        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) #Defining left front encoder
            counterFL += 1
        forward()
    pwm_right.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)
    return
def treverse(d_ticks):
    global counterBR,counterFL,pwm_left_b,pwm_right_b,buttonFL,buttonBR
    counterBR = 0
    counterFL = 0
    i=0
    while (counterBR < d_ticks or counterFL < d_ticks):
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12)) 
            counterBR += 1

        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) 
            counterFL += 1
        reverse()
    pwm_left_b.ChangeDutyCycle(0)
    pwm_right_b.ChangeDutyCycle(0)
    return
    
def tleft_ticks(angle_ticks):
    global counterBR,counterFL,buttonFL,buttonBR
    counterBR = 0
    counterFL = 0
    
    while (counterBR < angle_ticks or counterFL < angle_ticks):
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12)) 
            counterBR += 1

        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) 
            counterFL += 1
        turn_left()
    pwm_left_b.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)
    return

def tright_ticks(angle_ticks):
    
    global counterBR,counterFL,buttonBR,buttonFL
    counterBR = 0
    counterFL = 0
    while (counterBR < angle_ticks or counterFL < angle_ticks):
        
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1

        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) 
            counterFL += 1

        turn_right()
    pwm_right.ChangeDutyCycle(0)
    pwm_right_b.ChangeDutyCycle(0)
    return

def left(calc_turn):
    global pwm_left_b, pwm_left,speed
    current_imu = imu()
    if (current_imu == None):
        return None
    if (current_imu == 0):
        current_imu = 360
    ang_cal = current_imu - int(round(calc_turn))
    if(ang_cal < 0):
        ang_cal = 360 - abs(ang_cal)
    cturn = speed
    if(calc_turn >= 16):
        cturn = cturn
    elif(5 <= calc_turn <=15):
        cturn = cturn - 5
    angle_range = list(range(ang_cal-4,ang_cal+2))
    for i in range(len(angle_range)):
        if(angle_range[i] < 0):
            angle_range[i] = 360 - angle_range[i]
        elif(angle_range[i] > 360):
            angle_range[i] = abs(360 - angle_range[i])
    if(0<= current_imu <= 60):
        while(current_imu not in angle_range):
            pwm_left_b.ChangeDutyCycle(cturn)
            pwm_left.ChangeDutyCycle(cturn)
            current_imu = imu()
    else:
        while(current_imu >= ang_cal):
            pwm_left_b.ChangeDutyCycle(cturn)
            pwm_left.ChangeDutyCycle(cturn)
            current_imu = imu()
    current_imu = imu()
    if( 8 <= abs(current_imu - ang_cal) <=20):
        if((current_imu - ang_cal) < 0):
            angle_ticks = deg_ticks(abs(current_imu - ang_cal))
            tright_ticks(angle_ticks)
        else:
            angle_ticks = deg_ticks(abs(current_imu - ang_cal))
            tleft_ticks(angle_ticks)
    return
    
def right(calc_turn):
    global pwm_right, pwm_right_b,speed
    current_imu = imu()
    if (current_imu == None):
        return None
    ang_cal = current_imu + int(round(calc_turn))
    if(ang_cal > 360):
        ang_cal = abs(360 - ang_cal)
    cturn = speed
    if(calc_turn >= 16):
        cturn = cturn
    elif(5 <= calc_turn <=15):
        cturn = cturn - 5
    angle_range = list(range(ang_cal-1,ang_cal+5))
    for i in range(len(angle_range)):
        if(angle_range[i] < 0):
            angle_range[i] = 360 - angle_range[i]
        elif(angle_range[i] > 360):
            angle_range[i] = abs(360 - angle_range[i])
    if(270 <= current_imu <= 360):
        while(current_imu not in angle_range):
            pwm_right.ChangeDutyCycle(cturn)
            pwm_right_b.ChangeDutyCycle(cturn)
            current_imu = imu()
    else:
        while(current_imu <= ang_cal):
            pwm_right.ChangeDutyCycle(cturn)
            pwm_right_b.ChangeDutyCycle(cturn)
            current_imu = imu()
    current_imu = imu()
    if( 8 <= abs(current_imu - ang_cal) <=20):
        if((current_imu - ang_cal) < 0):
            angle_ticks = deg_ticks(abs(current_imu - ang_cal))
            tleft_ticks(angle_ticks)
        else:
            angle_ticks = deg_ticks(abs(current_imu - ang_cal))
            tright_ticks(angle_ticks)   
    return

def perform_action_turn(turn_degree , direction):
    global pwm_right, pwm_right_b, pwm_left_b, pwm_left
    if(direction == 'left'):
        calc_turn = abs(turn_degree)
        left(calc_turn)
        pwm_left_b.ChangeDutyCycle(0)
        pwm_left.ChangeDutyCycle(0)
    else :
        calc_turn = abs(turn_degree)
        right(calc_turn)
        pwm_right.ChangeDutyCycle(0)
        pwm_right_b.ChangeDutyCycle(0)

def degree_mapping(x):
    direction = ""
    degree = (x - (640/2))*0.061 #Mapping coordinates to degree
    if(degree < 0):
        direction = 'left'
    else:
        direction = 'right'
    return degree,direction

def close_gripper(): #Controlling servo
    global pwm_gripper
    pwm_gripper.ChangeDutyCycle(3.0)
    time.sleep(0.5)
    return
def open_gripper():
    global pwm_gripper
    pwm_gripper.ChangeDutyCycle(7.0)
    time.sleep(0.5)
    return

def turn_o (present_imu_change,pose):
    if( (present_imu_change - pose) < 0 ):
        angle_ticks = deg_ticks(abs(present_imu_change - pose))
        tright_ticks(angle_ticks)
    elif((present_imu_change - pose) > 0):
        angle_ticks = deg_ticks(abs(present_imu_change - pose))
        tleft_ticks(angle_ticks)

def image_dimen(width,x_center,focal_length,known_width):
    return (known_width*focal_length)/width
    
def move():
    global ang_distance,x_coord,y_coord
    move_reverse_first = dist_ticks(0.2)
    treverse(move_reverse_first)
    current_pose = imu()
    if( 0 <= current_pose <= 179):
        close_pose = 180 - current_pose
        angle_ticks = deg_ticks(close_pose)
        tright_ticks(angle_ticks)
    elif ( 181 <= current_pose <= 360):
        close_pose =  current_pose - 180
        angle_ticks = deg_ticks(close_pose)
        tleft_ticks(angle_ticks)
    if((abs(imu() - 180 )) > 3):
        turn_o (imu(),180)
    distance2 = ultrasonic_reading() - 0.5
    if(distance2 > 0 ):
        move_forward_after_180 = dist_ticks(distance2)
        
        tforward_ticks(move_forward_after_180)
        real_angle = abs(360-imu())
x_coord.append(float(dist_by_polar)*float(math.cos(math.radians(real_angle)))) #Storing positions
y_coord.append(float(dist_by_polar)*float(math.cos(math.radians(real_angle))))
    turn_to_270 = abs(270 - imu())
    angle_ticks = deg_ticks(turn_to_270)
    tright_ticks(angle_ticks)

    if((abs(imu() - 270 )) > 4):
        turn_o (imu(),270)
    distance3 = ultrasonic_reading() - 0.5

    if(distance3 > 0 ):
        move_forward_after_270 = dist_ticks(distance3)
        tforward_ticks(move_forward_after_270)
        real_angle = abs(360-imu())
        x_coord.append(float(dist_by_polar)*float(math.cos(math.radians(real_angle))))
        y_coord.append(float(dist_by_polar)*float(math.cos(math.radians(real_angle))))

    open_gripper()
    ang_distance.append(2.7)
    move_reverse_last = dist_ticks(0.2)
    ang_distance.append(-0.2)
    treverse(move_reverse_last)
    quarter_turn = 135
    angle_ticks = deg_ticks(quarter_turn)
    tright_ticks(angle_ticks)
    move_forward_after_place = dist_ticks(0.4)
    tforward_ticks(move_forward_after_place)
    ang_distance.append(0.4)
    return

def detect(image,low,high,j):
    global ang_distance
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masked_image= cv2.inRange(hsv,low,high)
    erode_image = cv2.erode(masked_image, None, iterations = 1)
    detected_image = cv2.bitwise_and(image,image, mask= masked_image)
    contours = cv2.findContours(erode_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    try:
        if(len(contours[0]) > 0):
            contour_list = imutils.detect(contours)
            sorted_contours = sorted(contour_list, key=cv2.contourArea, reverse=True)[:4]
            max_contour = max(sorted_contours, key = cv2.contourArea)
            ((max_x_center, max_y_center), max_radius) = cv2.minEnclosingCircle(max_contour)
            max_rect_contour_area = cv2.contourArea(max_contour)
            x, y, w, h = cv2.boundingRect(max_contour)
            M = cv2.moments(max_contour)
            if(M["m00"] != 0 ): 
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                center = (x + (w//2), y+(h//2))
            cv2.rectangle(image,(x,y), (x+w,y+h), (0,0,255), 2)
            cv2.circle(image,center,1,(255,0,0),2)
            return (image,max_x_center,w)
        else:
            if(j==0):
                d_ticks = dist_ticks(0.05)
                ang_distance.append(0.05)
                tforward_ticks(d_ticks)
                angle_ticks = deg_ticks(10)
                tleft_ticks(angle_ticks)
            else:
                angle_ticks = deg_ticks(10)
                tleft_ticks(angle_ticks)
    except:
        pass
    return (image,-1,-1)

x_coord = []
y_coord = []
ang_distance = []
image_list = []
x_coord.append(0)
y_coord.append(0)
ser = serial.Serial('/dev/ttyUSB0', 9600)
while (ser.in_waiting == 0):
    continue
ser.reset_input_buffer()

counterBR = np.uint64(0)
counterFL = np.uint64(0)
buttonBR = int(0)
buttonFL = int(0)
mov_speed = 40 
kp = 2
ki = 0.0002
kd = 0.5
prev_error = 0
t_error = 0
init()

trig = 16
echo = 18

pwm_gripper = gpio.PWM(36,50)
pwm_gripper.start(7.0) #Opening gripper
time.sleep(0.5)
speed = 45 #turning speed
pwm_left_b = gpio.PWM(33, 50)  
pwm_left = gpio.PWM(37, 50) 
val = 0
pwm_left_b.start(val)
pwm_left.start(val)
pwm_right = gpio.PWM(31, 50) #Left 
pwm_right_b = gpio.PWM(35, 50) #right
pwm_right.start(val)
pwm_right_b.start(val)
time.sleep(0.1)
count=0

known_width = 0.05
low_hsv_values = {'R': [146,79,73], 'G':[43,38,63], 'B':[68,113,94]}
high_hsv_values = {'R':[255,255,255], 'G':[100,255,255], 'B':[121,255,255]}
values = ['R','G','B','R','G','B','R','G','B']
def get_frames():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(0.01)
    video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ret, frame = video_capture.read()
    video_capture.release()
    return ret,frame
for j in range(9):
    current_color = values[j]
    low = np.array(low_hsv_values[current_color],np.uint8)
    high = np.array(high_hsv_values[current_color],np.uint8)
    loop_exit_count = 0
    while(1):
        ret,frame = get_frames()
        if(ret):
            frame = cv2.flip(frame,-1)
            img_cap,x_center,width = detect(frame,low,high,j)
            image_list.append(img_cap)
            if(x_center != -1):
                turn_degree , direction = degree_mapping(x_center)
                if(abs(turn_degree) > 5):
                    perform_action_turn(turn_degree , direction)
                    time.sleep(0.5)

                else:
                    distance_ = 0.2
                    focal_length = (width*distance_)/known_width
                    count = 5
                    
                    if(210< width):
                        block_distance = 0
                    elif(width<=194 and width >= 160):
                        ret,frame = get_frames()
                        frame = cv2.flip(frame,-1)
                        img_cap,x_center,width = detect(frame,low,high,j)
                        image_list.append(img_cap)
                        turn_degree , direction = degree_mapping(x_center)
                        if(abs(turn_degree) > 5):
                            perform_action_turn(turn_degree , direction)
                            d_ticks = dist_ticks(0.06)
                            ang_distance.append(-0.06)
                            treverse(d_ticks)
                        block_distance = 0.01
                    elif(width<=155 and width >= 100):
                        block_distance = 0.02
                        
                    elif(width<=99 and width >= 59):
                        block_distance = 0.05
                    elif(width<60):
                        block_distance = 0.20
                    else:
                        block_distance = 0.08
                    if(block_distance == 0):
                        
                        turn_degree , direction = degree_mapping(x_center)
                        if(abs(turn_degree) > 5):
                            perform_action_turn(turn_degree , direction)
                            d_ticks = dist_ticks(0.06)
                            ang_distance.append(-0.06)
                            treverse(d_ticks)
                        else:
                            close_gripper()
                            dist_by_polar = sum(ang_distance)
                            real_angle = abs(360-imu())
                            
                            x_coord.append(float(dist_by_polar)*float(math.cos(math.radians(real_angle))))
                            y_coord.append(float(dist_by_polar)*float(math.cos(math.radians(real_angle))))
                            ang_distance = []
                            ret,frame = get_frames()
                            frame = cv2.flip(frame,-1)
                            img_cap,x_center,width = detect(frame,low,high,j)
                            image_list.append(img_cap)
                            try:
                                email_send()
                            except:
                                print("Failed Email,{0}".format(j))
                                pass
                            print("Email Passed,{0}".format(j))
                            count = 0
                            move()
                            break
                    else:
                        d_ticks = dist_ticks(block_distance)
                        ang_distance.append(block_distance)
                        tforward_ticks(d_ticks)
                        ret,frame = get_frames()
                        frame = cv2.flip(frame,-1)
                        img_cap,x_center,width = detect(frame,low,high,j)
                        image_list.append(img_cap)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") :       
                break
            if(loop_exit_count == 60):
                break
            loop_exit_count+=1
gameover()
pwm_left_b.stop()
pwm_left.stop()
pwm_right.stop()
pwm_right_b.stop()
pwm_gripper.stop()
for i in range(len(image_list)):
    cv2.imwrite(f"images/image_4_{i}.jpg",image_list[i])
plt.title("Robot Trajectory ")
plt.xlabel('x axis(ft)')
plt.ylabel('y axi5s(ft)')
plt.plot(x_coord,y_coord,color = 'g')
plt.savefig("Robot_trajectory.png")
cv2.destroyAllWindows()
















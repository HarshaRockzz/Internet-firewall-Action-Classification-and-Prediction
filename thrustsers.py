import time
import serial
import smbus
import ms5837
import os
import sys, tty, termios, pigpio
from csv import writer
from datetime import datetime
#from Adafruit_ADS1x15 import ADS1115

#adc = ADS1115()
value = 0

# Servo ports declaration for PWM signals
servo1 = 4    # T1
servo2 = 22   # T2
servo3 = 18   # T3
servo4 = 17   # T4
dit = pigpio.pi()

current_datetime = datetime.now()
fnd = current_datetime.strftime("%b_%d_%Y_%H_%M_%S")
str_current_datetime = str(fnd)
file_name = str_current_datetime + ".csv"
file = open(file_name, 'w') 
print("File created : ", file.name)
file.close()

time.sleep(.1)
bus = smbus.SMBus(1)
sensor = ms5837.MS5837_30BA()
IMU_port = serial.Serial('/dev/ttyUSB1', 57600, timeout=5)
ser = serial.Serial('/dev/ttyUSB2', 9600, timeout=1)

yaw = 0
pitch = 0
roll = 0
pmbar = 0
depth = 0
temper = 0
latitude = 0
longitude = 0
al = 0
sp = 0
packet = "0"

def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted

def time_convert(sec):
    mins = sec // 60
    sec = sec % 60
    hours = mins // 60
    mins = mins % 60

def data_write():
    today_1 = datetime.today()
    dt_string = today_1.strftime("%d/%m/%Y %H:%M:%S")
    data = ',yaw'+','+str(yaw)+',pitch'+','+str(pitch)+',roll'+','+str(roll)+',pressure'+','+str(pmbar)+',depth'+','+str(depth)
    msg = 'Date:,' + str(dt_string) + data
    write_data = msg.split(',')
    with open(file_name, 'a') as f_object:
        writer_object = writer(f_object)
        writer_object.writerow(write_data)
        f_object.close()

z = 1
while z < 5:
    dit.set_servo_pulsewidth(servo1, 1500)
    dit.set_servo_pulsewidth(servo2, 1500)
    dit.set_servo_pulsewidth(servo3, 1500)
    dit.set_servo_pulsewidth(servo4, 1500)
    z = z + 1
    time.sleep(2)

count = 1
# Assume initial state variables for servo positions
initial_servo1_pulse = 1500
initial_servo2_pulse = 1500

# Flag to keep track of whether the AUV is avoiding an object
avoiding_object = False

# Variables to store previous state for returning after avoiding the object
previous_servo1_pulse = initial_servo1_pulse
previous_servo2_pulse = initial_servo2_pulse

# Distance to travel forward after detecting an object (in cm)
distance_to_travel = 20

# Flag to track whether the AUV is in the process of moving forward
moving_forward = False

# Time when the AUV started moving forward
forward_start_time = 0

# Duration to move forward (in seconds)
forward_duration = 3

ser = serial.Serial(
    port = '/dev/ttyUSB1',
    baudrate=9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout  = 1
)
ser.flush()
while True:
    try:
        data_write()
        print("entering the loop")
        time.sleep(0.5)
        line = IMU_port.readline().decode('utf-8') 
        if len(line) == 0:
            print("time out! ")
            sys.exit()
        print("IMU", line)
        IMU_info = line.split('=')
        print(IMU_info)
        IMU_data = IMU_info[1].split(',')
        yaw = IMU_data[0]
        pitch = IMU_data[1]
        roll = IMU_data[2]

        if not sensor.init():
            print("Sensor could not be initialized")
            exit(1)
        if sensor.read():
            pmbar = round(sensor.pressure(), 3)
            depth = round(sensor.depth(), 3)
            temper = round(sensor.temperature(), 3)
            print(f"Pressure: {pmbar} mbar, Depth: {depth:.3f} m, Temperature: {temper:.3f} F")

        yaw1 = float(yaw)
        print(yaw1)
        
        if count <= 70:
            msg_data = "1600,1600,1500,1500"       
            time.sleep(0.3)
            print("forward")
        elif count > 70 and count <= 150:
            msg_data = "1500,1500,1600,1600"
            print("divein")
            time.sleep(0.3)
        elif count > 150 and count <= 180:
            msg_data = "1500,1500,1400,1400"
            print("diveout")
            time.sleep(0.3)
        elif count > 180 and count <= 200:
            msg_data = "1500,1500,1500,1500"
            print("stop")
            time.sleep(0.3)
        else:
            msg_data = "1500,1500,1500,1500"
            time.sleep(0.01)
        
        count = count + 1
        print(count)
        
        T_data = msg_data.split(",")
        dit.set_servo_pulsewidth(servo1, T_data[0])
        time.sleep(0.1)
        dit.set_servo_pulsewidth(servo2, T_data[1])
        time.sleep(0.1)
        dit.set_servo_pulsewidth(servo3, T_data[2])
        time.sleep(0.1)
        dit.set_servo_pulsewidth(servo4, T_data[3])
        time.sleep(0.1)

        # Serial data reading and processing
        data = ser.readline().decode().rstrip()
        if data:
            re_data = data.split(',')
            print(f"Received data: {re_data}")
            conf = int(re_data[0])
            dist = float(re_data[1])
            width = float(re_data[2])
            height = float(re_data[3])
            ber_angle = float(re_data[4])
            print("Confidence=", conf)
            print("Distance=", dist)
            print("Width=", width)
            print("Height=" , height)
            print("Bearing angle=", ber_angle)
            
            if dist <= 10:  # Object detected within 10cm
                avoiding_object = True
                if ber_angle > 0:  # Object on the right side
                    # Adjust servo motors to move AUV right
                    # Example:
                    # Increase pulse width of left servo and decrease pulse width of right servo
                    dit.set_servo_pulsewidth(servo1, 1600)
                    dit.set_servo_pulsewidth(servo2, 1550)
                elif ber_angle < 0:  # Object on the left side
                    # Adjust servo motors to move AUV left
                    # Example:
                    # Increase pulse width of right servo and decrease pulse width of left servo
                    dit.set_servo_pulsewidth(servo1, 1550)
                    dit.set_servo_pulsewidth(servo2, 1600)
            else:
                if avoiding_object:
                    # Restore servo positions to previous state
                    dit.set_servo_pulsewidth(servo1, previous_servo1_pulse)
                    dit.set_servo_pulsewidth(servo2, previous_servo2_pulse)
                    avoiding_object = False
                    moving_forward = True
                    forward_start_time = time.time()
                # Additional actions based on distance and bearing angle can be added here
            
            # If AUV is in the process of moving forward
            if moving_forward:
                # Calculate elapsed time since starting to move forward
                elapsed_time = time.time() - forward_start_time
                if elapsed_time < forward_duration:
                    # Move forward
                    dit.set_servo_pulsewidth(servo1, 1600)
                    dit.set_servo_pulsewidth(servo2, 1600)
                else:
                    # Stop moving forward
                    moving_forward = False
                    # Update previous servo positions for returning after avoiding the object
                    previous_servo1_pulse = 1600
                    previous_servo2_pulse = 1600
                    forward_start_time = 0
                    
                    msg = 'Data received'
            ser.write(msg.encode())
        
    except Exception as e:
        print(f"An error occurred: {e}")
        continue

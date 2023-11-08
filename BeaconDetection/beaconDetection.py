#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
desVelL=int(0)
desVelR=int(0)
detection = False

if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    while True:
        # print('send left for ', leftMotor, ' and right: ', rightMotor)
        sendString('/dev/ttyACM0',115200,'<'+str(desVelL)+','+str(desVelR)+'>',0.0001)

        desVelL = 5
        desVelR = 5

        # if detection:
        #     wait(5sec)
        # else:


        # if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
                 
        #         line = ser.readline().decode('utf-8')
        #         line=line.split(',')
        #         #this splits the incoming string up by commas
        #         try:
        #             a=int(line[0])
        #             b=int(line[1])
        #             c=int(line[2])
        #             d=int(line[3])
        #             e=int(line[4])
        #             f=int(line[5])
        #             g=int(line[6])
        #             h=int(line[7])
        #             linePos=int(line[8]) #we dont convert this to a float becasue we went to be able to recieve the message that we are at a cross, which wont be an int. 
        #             # On straight line
        #             val = sum([a,b,c,d,e,f,g])

        #             temp_l = min(400,100 + sum([a,b,c,d]) * .023)
        #             temp_r = min(400,100 + sum([e,f,g,h]) * .023)
        #             leftMotor = temp_l
        #             rightMotor = temp_r


        #             if not hitFirst and sum([a,b,c,d,e,f,g,h]) > 7700:
        #                 print('hit first intersection')
                        
        #                 passing_first = time.time()
        #                 timer_on=True
        #                 hitFirst = True


        #             elif not hitSecond and hitFirst and (time.time() - passing_first > 1) and sum([a,b,c,d,e,f,g,h]) > 7700:
        #                 timer_on=True
                        
        #                 print(' at second intersection')
        #                 # leftMotor = 0
        #                 passing_first = time.time()
        #                 hitSecond = True
                       
        #             elif  hitSecond and timer_on == True:
        #                 # leftMotor = 0
        #                 # rightMotor = 0
        #                 leftMotor = 175
        #                 rightMotor = 0
        #                 print("Got Here, only left")
        #                 # time.sleep(.5)
        #                 # print('alright and we keep goig')
        #                 # timer_on = False
        #                 if((time.time() - passing_first > .60)):
        #                     timer_on = False
        #             # elif hitSecond and hitFirst:
        #             #      leftMotor = 200
        #             #      rightMotor = -200

        #             #     if not timer_on:
        #             #         passing_first = time.time()
        #             #         timer_on=True

        #             #     if time.time() - passing_first > 1: 
        #             #         hitSecond= True
        #             #         timer_on=False
        #             #         rightMotor=150
        #             #         leftMotor=150
                        
        #             if hitFirst and hitSecond and sum([a,b,c,d,e,f,g,h]) > 7000 and not timer_on:
        #                 print('THIRD')
        #                 leftMotor=0 
        #                 rightMotor=0
                         
        #             if(sum([a,b,c,d,e,f,g,h])>7000): 
        #                 print([a,b,c,d,e,f,g,h,linePos], 'and sum:', sum([a,b,c,d,e,f,g,h]), ' and hit first:', hitFirst, 'and hit second', hitSecond, 'and timer', timer_on)
        #         except KeyboardInterrupt:
        #             print('Interrupted')
        #         except Exception as e:
        #              pass
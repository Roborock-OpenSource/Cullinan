# -*- coding: utf-8 -*-
# coding=gbk
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.text as mt
import time
import thread
import math
import sys
import serial
import threading



#......Change the two parameters below...............
T = 1
SPOTANGLE = 188
#....................................................
packageLost = 0
pLost = 0
TT = 360*T
handle = "/dev/tty.usbmodem00000000001A1"
thread_should_close = 0
plt.polar()

def Loop():
    alpha = 0
    xa=[0]*TT
    ya=[0]*TT
    global thread_should_close
    global pLost,packageLost
    ser = serial.Serial(sys.argv[1] if len(sys.argv) > 1 else handle , sys.argv[2] if len(sys.argv) > 2 else 115200)
    nWaring = 0
    nErr = 0
    DataValid =[]
    DataLight = []
    Data = []
    
    while True:
        if thread_should_close == 1:
            break;
        
        start = int(ser.read().encode('hex'), 16)
        if start != 0xfa:
            packageLost = 1
            continue
        index = int(ser.read().encode('hex'), 16)
        if index < 0xa0 or index > 0xf9:
            packageLost = 1
            continue;
        
        if packageLost == 1:
            packageLost = 0
            pLost = pLost + 1
            #print "\33[93mPackage Lost: %05d\33[0m" %(pLost)
        
        speed = int(ser.read().encode('hex'), 16) + (int(ser.read().encode('hex'), 16) << 8)
        speed = speed*1.0/64
        for i in range(4):
            dist = int(ser.read().encode('hex'), 16) + (int(ser.read().encode('hex'), 16) << 8)
            intn = int(ser.read().encode('hex'), 16) + (int(ser.read().encode('hex'), 16) << 8)
            Angle = (index - 0xa0) * 4 + i
            valid = 0 if dist & 0x8000 != 0 else dist & ~0xc000
            warning = 1 if dist & 0x4000 != 0 else 0
            err = 0 if dist & 0x8000 == 0 else dist & 0xff
            hudu = math.radians(Angle)
            
            Data.append(valid)
            DataLight.append(intn)
            if err == 0:
                xa[alpha%TT]= hudu
                ya[alpha%TT]= valid
                DataValid.append(valid)
            else:
                xa[alpha%TT]= 0
                ya[alpha%TT]= 0
                nErr = nErr + 1
            if warning != 0:
                nWaring = nWaring + 1
                xa[alpha%TT]= 0
                ya[alpha%TT]= 0
            
            if alpha % TT == 0:
                N1 = float(len(DataValid))
                N2 = float(len(Data))
                maxLight = np.max(DataLight)
                minLight = np.min(DataLight)
                figtext = u"Error: %.2f%% \nWarning: %.2f%%\nmaxLight:%d\nminLight:%d\nSpeed: %.2f\nSample rate: %d\npLost: %4d" %(nErr*1.0/TT*100,nWaring*1.0/TT*100,maxLight,minLight,speed,TT,pLost)
                DataValid = []
                DataLight = []
                Data = []
                nWaring = 0
                nErr = 0
                plt.clf()
                plt.polar(xa,ya,'d')
                plt.figtext(0.9,0.7,figtext,ha = 'center',fontsize=15)
                plt.draw()
            alpha = alpha+1
            
            if err != 0:
                print "\33[91m%09d %03d: %5d %5d %d %#04x\33[0m" %(alpha ,Angle,valid, intn,warning,err)
            elif warning != 0:
                print "\33[95m%09d %03d: %5d %5d %d %#04x\33[0m" %(alpha ,Angle,valid, intn,warning,err)
            else:
                print "\33[92m%09d %03d: %5d %5d %d %#04x\33[0m" %(alpha ,Angle,valid, intn,warning,err)
    ser.close()

tsk = []
thread1 = threading.Thread(target = Loop)
thread1.start()
tsk.append(thread1)

plt.show()
thread_should_close = 1
for tt in tsk:
    tt.join()
plt.close()



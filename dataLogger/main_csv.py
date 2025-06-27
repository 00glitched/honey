import matplotlib.pyplot as plt
import serial
import time
import os
import csv


PORT="/dev/ttyUSB0"
BAUDRATE=115200
MAXIT=5000

init_time = time.time()
usb = serial.Serial(PORT, BAUDRATE)


def save(inp):
    pass

def addtoplot(inp):
    pass

def process(inp):
    save(inp)
    addtoplot(inp)

count=0
canread=False
noSTOP=True

while noSTOP:
    data = str(usb.readline())
    #process(data)
    data = data[2:len(data)-5]
    time_i = time.time()-init_time

    timedata=""
    if count>0:
        timedata=str(time_i)+";"+data
    else:
        timedata=data

    if count>MAXIT:
        noSTOP=False
    
    #pos=0
    
    #sdata=""
    #if canread==True:
    #    sdata+="{"
    #for i in range(len(data)):
    #    if canread==True:
    #        if data[i]==':':
    #            sdata += "'" +data[pos+1:i] + "':"
    #            pos=i+1
    #        if data[i]==',':
    #            sdata += data[pos:i] +","
    #            pos=i+1
    #        if data[i]==';':
    #            sdata += data[pos:i] +"}"
    #    else:
    #        if data[i]==';':
    #            canread=True
        
    #print(data)
    #print(sdata)
    #ddata=eval(sdata)
    #print(ddata)
    print(timedata)

    with open('temperature.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        DATA = timedata.split(";")
        writer.writerow(DATA)
        file.close()
    
    count+=1

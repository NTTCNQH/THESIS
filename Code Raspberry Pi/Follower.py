import time
import datetime
import serial
import math
import socket

HOST='192.168.1.142'
PORT=1233

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

full_msg = ''
while True:
    msg = s.recv(1024)
    if len(msg) <= 0:
        break
    full_msg += msg.decode("utf-8")

print(full_msg)


ser=serial.Serial("/dev/ttyACM0",9600)
ser.baurate=9600
time.sleep(3)

l=0.22
r=0.0425
d=0.6
Kv=7.25
Kw=0.205
vL=0.12
wL=0.0
phi=0
beta=0.0
thetaL=0.0
xL=0.0
yL=0.0
thetaF=0.0
xF=-0.6
yF=-1.2

def callw():
    global vL,wL,current
    current=datetime.datetime.utcnow()
    time_passed=current-start
#    if time_passed.total_seconds() >= 10 and time_passed.total_seconds() <= 30:
#        wL=0.2
#    elif time_passed.total_seconds() >= 6 and time_passed.total_seconds() <= 8:
#        wL=-0.4
    if time_passed.total_seconds() > 24:
        wL=0.0
        vL=0.0
#    else: wL=0.0

line1=''
line2=''
wr_real='0'
wl_real='0'
wF_real=0.0
vF_real=0.0
start=datetime.datetime.utcnow()
lasttime=datetime.datetime.utcnow()

while True:
    #time.sleep(0.001)
    callw()
    Ts=datetime.datetime.utcnow()-lasttime
    if Ts.total_seconds()>0.005:
        thetaL=thetaL+wL*(datetime.datetime.utcnow()-lasttime).total_seconds()
        xL=xL+vL*math.cos(thetaL)*(datetime.datetime.utcnow()-lasttime).total_seconds()
        yL=yL+vL*math.sin(thetaL)*(datetime.datetime.utcnow()-lasttime).total_seconds()
        thetaF=thetaF+wF*(datetime.datetime.utcnow()-lasttime).total_seconds()
        xF=xF+vF*math.cos(thetaF)*(datetime.datetime.utcnow()-lasttime).total_seconds()
        yF=yF+vF*math.sin(thetaF)*(datetime.datetime.utcnow()-lasttime).total_seconds()
        lasttime=datetime.datetime.utcnow()
    beta=thetaL-thetaF
    d_real=math.sqrt((xL-xF)**2+(yL-yF)**2)
    phi_real=math.atan2((yL-yF),(xL-xF))
    Er=d_real*math.cos(phi_real)-d*math.cos(math.atan2(math.sin(phi),math.cos(phi)))
    Ev=d_real*math.sin(phi_real)-d*math.sin(math.atan2(math.sin(phi),math.cos(phi)))
    vF=vL*math.cos(beta-phi)/math.cos(phi)+Er*Kv
    if vF>0.55:
        vF=0.55
    if vF<0:
        vF=0
    wF=vL*math.sin(beta)/(d*math.cos(phi))+Ev*Kw
    if wF>0.6:
        wF=0.6
    if wF<-0.6:
        WF=-0.6
    wr=vF/r+(l*wF)/(2*r)
    wl=vF/r-(l*wF)/(2*r)
    gui=str(wr)+','+str(wl)+'\n'
    ser.write(gui.encode())
    read_ser=str(ser.readline())
    if line1!=read_ser[2:][:-5]:
        line1=read_ser[2:][:-5]
        [wr_real,wl_real]=line1.split(',')
    vF_real=(r*(float(wr_real)+float(wl_real)))/2
    wF_real=(r*(float(wr_real)-float(wl_real)))/l
    giatri=str("%.2f" % vF)+','+str("%.2f" % wF)+','+str("%.2f" % thetaL)+','+str("%.2f" % thetaF)
    if line2!=giatri:
        line2=giatri
        print(line2)
    
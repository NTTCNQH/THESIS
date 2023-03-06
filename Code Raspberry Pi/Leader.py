import time
import datetime
import serial
import socket

HOST = '192.168.1.142'
PORT = 1234

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(5)
conn, addr = s.accept()
with conn:
    print(f"Connection from {addr} has been astablished!")
    conn.sendall(b'welcome to the server!')
    conn.close()


ser=serial.Serial("/dev/ttyACM0",9600)
ser.baudrate=9600
time.sleep(3)

l=0.22
r=0.0425
v=0.25
w=0.0
theta=0.0

def callw():
    global v,w,current_w
    current_w=datetime.datetime.utcnow()
    time_passed=current_w-start_w
    if time_passed.total_seconds() >= 2 and time_passed.total_seconds() <= 4:
        w=0.4
    elif time_passed.total_seconds() >= 6 and time_passed.total_seconds() <=8 :
        w=-0.4
    elif time_passed.total_seconds() >= 11.5:
        w=0.0
        v=0.0
    else: w=0.0

line=''
line2=''
wr_real='0'
wl_real='0'
start_w=datetime.datetime.utcnow()
lasttime=datetime.datetime.utcnow()

while True:
    callw()
    delaytime=datetime.datetime.utcnow()-lasttime
    if delaytime.total_seconds() > 0.005:
        theta=theta+delaytime.total_seconds()*w_real
        lasttime=datetime.datetime.utcnow()
    wr=v/r+(l*w)/(2*r)
    wl=v/r-(l*w)/(2*r)
    gui=str(wr)+','+str(wl)+'\n'
    ser.write(gui.encode())
    read_ser=str(ser.readline())
    if line!=read_ser[2:][:-5]:
        line=read_ser[2:][:-5]
        [wr_real,wl_real]=line.split(',')
    v_real=(r*(float(wr_real)+float(wl_real)))/2
    w_real=(r*(float(wr_real)-float(wl_real)))/l
    giatri=str("%.2f" % v_real)+','+str("%.2f" % w_real)+','+str("%.2f" % theta)
    if line2!=giatri:
        line2=giatri
        print(line2)
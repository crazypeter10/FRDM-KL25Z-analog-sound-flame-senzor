from PyQt5 import QtWidgets, QtCore, QtGui
from pyqtgraph import PlotWidget, plot
from PyQt5.Qt import Qt
import pyqtgraph as pg
import sys 
import os
from random import randint
import serial.tools.list_ports
from PyQt5.QtWidgets import *
from PyQt5 import QtGui,QtCore

import re

input_buff= [0] * 5
serialInstance=serial.Serial()
port="COM6"
serialInstance.baudrate=115200
serialInstance.port=port
serialInstance.open()

class MainWindow(QtWidgets.QMainWindow):

    decibels=0
    temperature=0
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        self.widget = QWidget()
        layout = QGridLayout()
        self.widget.setLayout(layout)

        btn_switch = QPushButton('Switch RGB LED')
        btn_switch.setStyleSheet("background-color: #4CAF50; border: none;color: white;padding: 15px 32px;text-align: center;text-decoration: none;font-size: 16px;margin: 4px 2px;")
        btn_switch.clicked.connect(self.switch_rgb_sensor)
        
        self.graphWidget_sound = pg.PlotWidget()
        self.graphWidget_flame = pg.PlotWidget()
        
        layout.addWidget(self.graphWidget_sound, 0, 0)
        layout.addWidget(self.graphWidget_flame, 0, 1)
        layout.addWidget(btn_switch, 1, 0)

        self.setCentralWidget(self.widget)
        #self.setCentralWidget(self.graphWidget)
        self.setWindowTitle("Detect decibels from sound sensor")

        #icon
        scriptDir = os.path.dirname(os.path.realpath(__file__))
        self.setWindowIcon(QtGui.QIcon(scriptDir + os.path.sep + 'sound.png'))

        self.xs = list(range(100))  # 100 time points
        self.ys = [30 for _ in range(100)]  # 100 data points
        
        self.xf = list(range(100))  # 100 time points
        self.yf = [30 for _ in range(100)]  # 100 data points

        self.graphWidget_sound.setBackground('k')
        self.graphWidget_flame.setBackground('k')

        pen = pg.mkPen(color=(0, 255, 255))
        self.data_line =  self.graphWidget_sound.plot(self.xs, self.ys, pen=pen)

        pen2 = pg.mkPen(color=	(255,0,0))
        self.data_line2 =  self.graphWidget_flame.plot(self.xf, self.yf, pen=pen2)
    
    def update_plot_data_sound(self):
        self.xs = self.xs[1:]  # Remove the first y element.
        self.xs.append(self.xs[-1] + 1)  # Add a new value 1 higher than the last.

        self.ys = self.ys[1:]  # Remove the first 
        self.ys.append(self.decibels)  
        #print(self.decibels)
        if self.decibels>0 and self.decibels<40:
           color1 = pg.mkPen(color=(0,255,0))    
           self.data_line.setData(self.xs, self.ys, pen = color1)  # Update the data.
        elif self.decibels>40 and self.decibels<60:
            # daca e valoare mica => GALBEN (255,255,0)
            color2 = pg.mkPen(color=(255,255,0))    
            self.data_line.setData(self.xs, self.ys, pen=color2)
        elif self.decibels>60:
            # daca e valoare mica => RED (255,0,0)
            color3 = pg.mkPen(color=(255,0,0))    
            self.data_line.setData(self.xs, self.ys, pen=color3)
            # Update the data.
    def update_plot_data_flame(self):
        self.xf = self.xf[1:]  # Remove the first y element.
        self.xf.append(self.xf[-1] + 1)  # Add a new value 1 higher than the last.

        self.yf = self.yf[1:]  # Remove the first 
        self.yf.append(self.temperature)  
        #print(self.decibels)
        self.data_line2.setData(self.xf, self.yf)  # Update the data.

    def switch_rgb_sensor(self):
        global serialInstance
        #print("switch rgb")
        serialInstance.write(b'a')
        
read_data=1
i=0

def Average(lst):
    return sum(lst) / len(lst)

def get_data_from_serial():
    global read_data
    global serialInstance
    global window
    global data
    global i
    if serialInstance.in_waiting:
        packet=serialInstance.readline()
       
        buffer=packet.decode('utf',errors='replace').strip("\n")
       
        
        #
        if buffer[1]=='S':
            
            
            #print(buffer)
            buffer=buffer[2:]
            buff=([int(s) for s in buffer.split() if s.isdigit()][0])
            data=buff
            
            #input_buff.append(data)
            #input_buff.pop(0)
            #i=i+1
            
            #print(input_buff)
            #average = Average(input_buff)
            #if (input_buff[3]-data)<50 :
            window.decibels=data
          
                
            #if i==10000:
            #    i=0
            
            
            
            
        else :
            #print(buffer)
            buffer=buffer[2:]
            buff=([int(s) for s in buffer.split() if s.isdigit()][0])
            data=buff

            window.temperature=data
            
            
            
        
        

data=0



app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()

timer = QtCore.QTimer()
timer2 = QtCore.QTimer()

timer.setInterval(50)
timer2.setInterval(1)
timer2.timeout.connect(get_data_from_serial)
#window.decibels=data
timer.timeout.connect(window.update_plot_data_sound)
timer.timeout.connect(window.update_plot_data_flame)
timer2.start()
timer.start()


    
sys.exit(app.exec_())
            
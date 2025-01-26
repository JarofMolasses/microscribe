# TODO: mate you gotta define some classes
# template: https://github.com/precise-simulation/mesh-viewer/blob/master/meshviewer_mpl_tk.py#L296-L298
# App class  
# View class        (render elements)
# Controller class  (GUI elements)

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D

import tkinter as tk
from tkinter import *
from tkinter.ttk import *

import time
from queue import Queue
import serial

# only display last [...] points
num_xyz_points = 64
tip_points = 1
x = []
y = []
z = []
tipx = []
tipy = []
tipz = []

plt.style.use('dark_background') # Dark theme
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
graph = ax.scatter([],[],[], color='aquamarine', alpha = 0.3,zorder=2)
graph2 = ax.scatter([],[],[], color='turquoise', alpha = 1,s=50,zorder=1)
text1 = fig.text(0, 0, "XYZ DATA", va='bottom', ha='left',color='lightgrey',fontsize=12)  # for debugging
text2 = fig.text(0.5,0.95, "XYZ DATA", va='top', ha='center',color='lightgrey',fontsize=32)  # for debugging

def init_plot():
    print("Init plot")
    ax.cla()
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.tick_params(axis='x', colors='grey')
    ax.tick_params(axis='y', colors='grey')
    ax.tick_params(axis='z', colors='grey')
    ax.set_xlabel("X (mm)", color ='grey')
    ax.set_ylabel("Y (mm)", color = 'grey')
    ax.set_zlabel("Z (mm)", color ='grey')
    ax.yaxis._axinfo["grid"]['linewidth'] = 0.1
    ax.xaxis._axinfo["grid"]['linewidth'] = 0.1
    ax.zaxis._axinfo["grid"]['linewidth'] = 0.1
    ax.set_xlim3d(-500,500)
    ax.set_ylim3d(-500,500)
    ax.set_zlim3d(-50, 300)

# see original: https://stackoverflow.com/questions/50342300/animating-3d-scatter-plot-using-python-mplotlib-via-serial-data
def update_lines(i):
    # I'm not using the iterator argument because my data is streaming from the serial port

    # use this block for clearing and redrawing. Not used with _offsets3d()
    curxlim3d = ax.get_xlim()
    curylim3d = ax.get_ylim()
    curzlim3d = ax.get_zlim()
    ax.cla()
    ax.set_xlim3d(curxlim3d)
    ax.set_ylim3d(curylim3d)
    ax.set_zlim3d(curzlim3d)

    packet_received = False
    while(not packet_received):
        try:
            serial_rx = ser.readline()
            try:
                data = str(serial_rx[0:len(serial_rx)-2].decode("utf-8"))
                if(data.startswith("XYZ")):
                    xyz = data.split(",")
                    #print(xyz)
                    dx = float(xyz[1])
                    dy = float(xyz[2])
                    dz = float(xyz[3])    
                    text1.set_text("{:d} points in memory".format(len(x)))  # for debugging
                    text2.set_text("[X,Y,Z]: [{:.2f}, {:.2f}, {:.2f}]".format(dx, dy, dz))  # for debugging
                    packet_received = True
                    x.append(dx)
                    y.append(dy)
                    z.append(dz)
                    tipx.append(dx)
                    tipy.append(dy)
                    tipz.append(dz)
                    if len(x) > num_xyz_points:
                        x.pop(0)
                        y.pop(0)
                        z.pop(0)
                    if len(tipx) > tip_points:
                        tipx.pop(0)
                        tipy.pop(0)
                        tipz.pop(0)
                    try:
                        #  You can update the plot with new data only
                        # graph._offsets3d = (x, y, z)
                        # graph2._offsets3d = (tipx,tipy,tipz)
                        
                        #  Or, if you can take the computation hit, redraw the whole plot. This way I get to draw lines too
                        graph = ax.plot(x,y,z, color='aquamarine', alpha = 0.3)
                        graph2 = ax.scatter(dx,dy,dz,color='turquoise', alpha = 1,s=40)
                    except:
                        pass
                    return graph,graph2
                else:
                    pass
            except UnicodeDecodeError:
                pass

        except serial.SerialException as e:
        #There is no new data from serial port
            pass
        except TypeError as e:
            #Disconnect of USB->UART occured
            pass

ser = serial.Serial('COM8', 9600, timeout = 5)
if(not ser.is_open):
    ser.open()
ser.flushInput()
time.sleep(2)
ser.write('c'.encode('utf-8'))      # turn on continuous reporting if not already enabled

def home_arm():
    print("Home")
    ser.write('h'.encode('utf-8'))

# Clear the data and restart the animation
# idk how to make this work. It runs at the beginning when I don't call it, and then when I call it it does nothing.
def reset_plot(anim):
    print("Reset plot")
    anim.pause()
    x.clear()
    y.clear()
    z.clear()
    tipx.clear()
    tipy.clear()
    tipz.clear()
    init_plot()

root = tk.Tk()
root.title("Microscribe 3D demo")

init_plot()
# Creating the Animation object
ani = animation.FuncAnimation(fig, update_lines, fargs = [], frames=100, interval=50, blit=False)
#plt.show()

menubar = Menu(root)
# microscribe options
msmenu = Menu(menubar, tearoff = 0)
menubar.add_cascade(label='Microscribe options', menu = msmenu)
msmenu.add_command(label = 'Home', command = home_arm)
# view options
view = Menu(menubar, tearoff = 0)
menubar.add_cascade(label = 'View options', menu = view)
view.add_command(label = 'Reset plot', command = reset_plot(anim = ani))

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
canvas.mpl_connect('button_press_event', ax._button_press)
canvas.mpl_connect('button_release_event', ax._button_release)
canvas.mpl_connect('motion_notify_event', ax._on_move)


def close_window():
    root.quit() # this doesn't seem to normally run and is needed to close properly
    root.destroy()
    return
root.protocol("WM_DELETE_WINDOW", close_window)

root.config(menu = menubar) 
root.mainloop()



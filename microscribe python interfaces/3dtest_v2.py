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

import threading


class plotData():
    def __init__(self):
        self.remember_xyz_points = 64               
        self.path_points = self.remember_xyz_points  # number of points to draw path of tip (ax.plot). set to 1 to hide path.
        self.tip_points = 1                          # number of points to show only the tip location (ax.scatter)
        self.csv_points = 512                        # number of points to save to csv 
        self.x = []
        self.y = []
        self.z = []
        self.tipx = []
        self.tipy = []
        self.tipz = []
        self.savex = []
        self.savey = []
        self.savez = []
        self.showPath = True
    
    def set_num_points(self, num_points):
        self.remember_xyz_points = num_points
        self.path_points = num_points

    def toggle_path(self):
        self.showPath = not self.showPath
        if(not self.showPath):
            self.path_points = 1
        else:
            self.path_points = self.remember_xyz_points
    
    def clearcsv(self):
        self.savex = []
        self.savey = []
        self.savez = []
        
    def cleardisplay(self):
        self.x = []
        self.y = []
        self.z = []
        self.tipx = []
        self.tipy = []
        self.tipz = []

class Arm():
    def __init__(self):
        self.ser = serial.Serial('COM8',9600, timeout=5)
    def open_source(self):
        if(not self.ser.is_open):
            self.ser.open()
            self.ser.flushInput()
            time.sleep(2)
            self.ser.write('c'.encode('utf-8'))      # turn on continuous reporting if not already enabled

    def home_arm(self):
        print("Home arm")
        self.ser.write('h'.encode('utf-8'))

    def open_source(self):
        if(not self.ser.is_open):
            self.ser.open()
            self.ser.flushInput()
            time.sleep(2)
            self.ser.write('c'.encode('utf-8'))      # turn on continuous reporting if not already enabled

class View():
    def __init__(self, arm = None, plotdata = None):
        if arm is None:
            arm = Arm()
        self.arm = arm

        if plotdata is None:
            plotdata = plotData()
        self.data = plotdata

        self.plot = plt
        self.plot.style.use('dark_background')

        self.fig = self.plot.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection = "3d")

        self.graph = self.ax.scatter([],[],[], color='aquamarine', alpha = 0.3,zorder=2)
        self.graph2 = self.ax.scatter([],[],[], color='turquoise', alpha = 1,s=50,zorder=1)
        self.graph3 = self.ax.scatter([],[],[], color='red', alpha = 1,s=50,zorder=1)

        self.text1 = self.fig.text(0, 0, "NUMBER OF POINTS", va='bottom', ha='left',color='lightgrey',fontsize=12)  # for debugging
        self.text2 = self.fig.text(0.5,0.95, "XYZ DATA", va='top', ha='center',color='lightgrey',fontsize=32)
        pass

    def init_plot(self):
        print("Init plot")
        self.ax.cla()
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.tick_params(axis='x', colors='grey')
        self.ax.tick_params(axis='y', colors='grey')
        self.ax.tick_params(axis='z', colors='grey')
        self.ax.set_xlabel("X (mm)", color ='grey')
        self.ax.set_ylabel("Y (mm)", color = 'grey')
        self.ax.set_zlabel("Z (mm)", color ='grey')
        self.ax.yaxis._axinfo["grid"]['linewidth'] = 0.1
        self.ax.xaxis._axinfo["grid"]['linewidth'] = 0.1
        self.ax.zaxis._axinfo["grid"]['linewidth'] = 0.1
        self.ax.set_xlim3d(-500,500)
        self.ax.set_ylim3d(-500,500)
        self.ax.set_zlim3d(-50, 300)
        
    def update_lines(self, i=1):
        # see original: https://stackoverflow.com/questions/50342300/animating-3d-scatter-plot-using-python-mplotlib-via-serial-data
        # I'm not using the iterator argument because my data is streaming from the serial port

        # use this block for clearing and redrawing. Not used with _offsets3d()
        curxlim3d = self.ax.get_xlim()
        curylim3d = self.ax.get_ylim()
        curzlim3d = self.ax.get_zlim()
        self.ax.cla()
        self.ax.set_xlim3d(curxlim3d)
        self.ax.set_ylim3d(curylim3d)
        self.ax.set_zlim3d(curzlim3d)
        self.ax.tick_params(axis='x', colors='grey')
        self.ax.tick_params(axis='y', colors='grey')
        self.ax.tick_params(axis='z', colors='grey')
        self.ax.set_xlabel("X (mm)", color ='grey')
        self.ax.set_ylabel("Y (mm)", color = 'grey')
        self.ax.set_zlabel("Z (mm)", color ='grey')
        self.ax.yaxis._axinfo["grid"]['linewidth'] = 0.1
        self.ax.xaxis._axinfo["grid"]['linewidth'] = 0.1
        self.ax.zaxis._axinfo["grid"]['linewidth'] = 0.1

        packet_received = False
        while(not packet_received):
            try:
                serial_rx = self.arm.ser.readline()
                try:
                    data = str(serial_rx[0:len(serial_rx)-2].decode("utf-8"))
                    if(data.startswith("XYZ")):
                        xyz = data.split(",")
                        #print(xyz)
                        dx = float(xyz[1])
                        dy = float(xyz[2])
                        dz = float(xyz[3])    
                        self.text1.set_text("{:d} points in memory".format(len(self.data.x)))  # for debugging
                        self.text2.set_text("(X,Y,Z): ({:.2f}, {:.2f}, {:.2f})".format(dx, dy, dz))  # for debugging
                        packet_received = True
                        self.data.x.append(dx)
                        self.data.y.append(dy)
                        self.data.z.append(dz)
                        self.data.tipx.append(dx)
                        self.data.tipy.append(dy)
                        self.data.tipz.append(dz)
                        if len(self.data.x) > self.data.path_points:
                            self.data.x.pop(0)
                            self.data.y.pop(0)
                            self.data.z.pop(0)
                        if len(self.data.tipx) > self.data.tip_points:
                            self.data.tipx.pop(0)
                            self.data.tipy.pop(0)
                            self.data.tipz.pop(0)
                        try:
                            #  You can update the plot with new data only
                            # graph._offsets3d = (x, y, z)
                            # graph2._offsets3d = (tipx,tipy,tipz)
                            
                            #  Or, if you can take the computation hit, redraw the whole plot. This way I get to draw lines too
                            self.graph = self.ax.plot(self.data.x,self.data.y,self.data.z, color='aquamarine', alpha = 0.3)
                            self.graph2 = self.ax.scatter(dx,dy,dz,color='turquoise', alpha = 1,s=40)
                            self.graph3
                        except:
                            pass
                        return self.graph,self.graph2
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


class GUI():
    def __init__(self, view = None, arm = None):
        
        if view is None:
            view = View()
        self.view = view

        if arm is None:
            arm = Arm()
        self.arm = arm

        root = tk.Tk()
        root.title("Microscribe 3D demo")
        
        self.view.init_plot()
        #self.ani = animation.FuncAnimation(self.view.fig, self.view.update_lines, fargs = [], frames=100, interval=50, blit=False)     # this works. But it's not as flexible and controlling it myself
        self.ani = None

        menubar = Menu(root)
        # microscribe options
        msmenu = Menu(menubar, tearoff = 0)
        menubar.add_cascade(label='Microscribe options', menu = msmenu)
        msmenu.add_command(label = 'Home', command = self.arm.home_arm)
        # view options
        view = Menu(menubar, tearoff = 0)
        menubar.add_cascade(label = 'View options', menu = view)
        view.add_command(label = 'Reset data', command = self.reset_plot)
        view.add_checkbutton(label = 'Freeze', command = self.render_toggle)
        view.add_checkbutton(label = 'Hide path', command = self.toggle_path)

        canvas = FigureCanvasTkAgg(self.view.fig, master=root)
        canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        canvas.mpl_connect('button_press_event', self.view.ax._button_press)
        canvas.mpl_connect('button_release_event', self.view.ax._button_release)
        canvas.mpl_connect('motion_notify_event', self.view.ax._on_move)
    
        self.root = root
        self.canvas = canvas
        self.update = True
                
        def close_window():
            root.quit() # this doesn't seem to normally run and is needed to close properly
            root.destroy()
            return
        root.protocol("WM_DELETE_WINDOW", close_window)

        self.root.config(menu = menubar)

    def render(self):
        if(self.update):
            self.view.update_lines()            # update plot
            self.canvas.draw()               # send updates to tkinder window
        self.root.after(50, self.render)     # HELL YEAH. Don't use the mpl animation function if we're using tkinter anyway

    def render_toggle(self):
        self.update = not self.update

    def reset_plot(self):
        print("Reset plot")
        # self.view.data.x.clear()
        # self.view.data.y.clear()
        # self.view.data.z.clear()
        # self.view.data.tipx.clear()
        # self.view.data.tipy.clear()
        # self.view.data.tipz.clear()
        self.view.data.cleardisplay()
        self.view.data.clearcsv()
        self.view.init_plot()
        self.view.update_lines()            # update plot
        self.canvas.draw()               # send updates to tkinder window
    
    def toggle_path(self):
        self.view.data.toggle_path()
        self.reset_plot()
    
class App():
    def __init__(self, arm = None, view = None, gui = None):
        if arm is None:
            arm = Arm()
        if view is None:
            view = View(arm)
        if gui is None:
            gui = GUI(view, arm)

        self.gui = gui
        self.view = view
        self.arm = arm

if __name__ == "__main__":
    app = App()
    app.arm.open_source()
    app.gui.render()
    app.gui.root.mainloop()

# TODO:
# We need to update just the artists and not the axes every time. cla() is SUPER SLOW
# Extract machine parameters automatically during autoscan(). Currently hardcoded
# Stop calculating matrices on Arduino and just report angles for numpy
# Implement background computation in separate thread (e.g. Serial, matrix manipulation). DONE

# TODO but next time.
# Once we get the machine params and perform calculations in numpy, then we can replace the Arduino with a simple FT232 serial-USB. Build a custom cable
# We have pushed matplotlib's 3D capabilities pretty much as far as they'll go. We can move to something actually resembling a 3D engine (vispy, plotly, pyvista, MayaVi, pyQtgraph, three.js...)

# MATPLOTLIB 3.10.0
# Python 3.11

import numpy as np
# from scipy.spatial.transform import Rotation as R
# from scipy.interpolate import griddata
import math
import random

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# from pydub import AudioSegment
# from pydub.playback import play
# CMM_beep = AudioSegment.from_wav("bamboo.wav")

import pygame
pygame.init()
CMM_beep = pygame.mixer.Sound("cmmbeep2.wav")
CMM_beep.set_volume(0.3)
bamboo_sound = pygame.mixer.Sound("bamboo.wav")
bamboo_sound.set_volume(0.5)

#mpl.use("Agg")
mpl.use("Qt5Agg")                        # Qt5 FIXES THE .set() method on labels. using .set() on checkbuttons is still broken 
import mpl_toolkits.mplot3d
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D,art3d

import tkinter as tk
from tkinter import Menu, Button, Frame
from tkinter.filedialog import asksaveasfile
from tkinter.scrolledtext import ScrolledText

# this seems good for customizing certain widgets. I've got a weird mix of traditional tk and ttkbootstrap just for the scrollbar to look better
import ttkbootstrap as tb
from ttkbootstrap.scrolled import ScrolledText as ST

import time
import timeit
import serial
import sys
import glob

import DHjoints
from DHjoints import Robot,Link,LinkRender

from threading import Thread
from queue import Queue

planecolor = 'plum'
P2Pcolor = 'skyblue'
P2Plcolor = 'palegreen'
readoutcolor = 'gainsboro'
indicatorcolor = 'gainsboro'
consolecolor = 'slategrey'
styluscolor = 'slategrey'
tipcolor = 'aquamarine'
cloudcolor = 'red'
axiscolor = 'grey'
linkcolor = 'slategrey'
jointcolor = 'lightgrey'

STYLE_CRT_GREEN = 'CRT_GREEN'
STYLE_CRT_AMBER = 'CRT_AMBER'
style = STYLE_CRT_GREEN
if(style == STYLE_CRT_GREEN):
    planecolor = 'mediumaquamarine'
    P2Pcolor = 'lightblue'
    P2Plcolor = 'orangered'
    readoutcolor = 'palegreen'
    indicatorcolor = 'palegreen'
    consolecolor = 'palegreen'
    styluscolor = 'palegreen'
    tipcolor = 'aquamarine'
    cloudcolor = 'red'
    axiscolor = 'palegreen'
    linkcolor = 'palegreen'
    jointcolor = 'palegreen'

elif(style == STYLE_CRT_AMBER):
    planecolor = 'bisque'
    P2Pcolor = 'skyblue'
    P2Plcolor = 'palegreen'
    readoutcolor = 'darkorange'
    indicatorcolor = 'darkorange'
    consolecolor = 'darkorange'
    styluscolor = 'darkorange'
    tipcolor = 'orangered'
    cloudcolor = 'red'
    axiscolor = 'darkorange'
    linkcolor = 'darkorange'
    jointcolor = 'darkorange'

NUM_ENCODERS = 5        # Number of encoders to read. Do we really need this here

# Shared queue commands
START_WORKER = 'start'
WORKER_RUNNING = 'started'
STOP_WORKER = 'stop'
WORKER_STOPPED = 'stopped'
HOME_ARM = 'home'
serial_worker_in_queue = Queue()           # shared queue to issue commands to serial thread
serial_worker_out_queue = Queue()             # shared queue to get data from serial thread 

# Storing keybinds for easy remapping later
class Keys:
    plane = 'q'
    point = 'p'                 
    rotate = '2'            # Rotate must be attached to mouse.
    pan = 'Tab'
    cloud = 'space'
    xy = 'x'
    xz = 'y'
    yz = 'z'
    freeze = 'f'

class PrintLogger(object):  # create file like object

    def __init__(self, textbox):  # pass reference to text widget
        self.textbox = textbox  # keep ref

    def write(self, text):
        #self.textbox.configure(state="normal")  # make field editable --used for tk scrolledtext, not used for ttkbootstrap version
        self.textbox.text.configure(state="normal")
        self.textbox.insert("end", text)  # write text to textbox
        self.textbox.see("end")  # scroll to end
        #self.textbox.configure(state="disabled")  # make field readonly -- used for tk scrolledtext, not used for ttkbootstrap version
        self.textbox.text.configure(state="disabled")
        self.textbox.update_idletasks()             # should make it update
    
    def close():
        pass

    def flush(self):  # needed for file like object
        pass

# implementing point to reference plane or point to point measurement
# TODO later: evaluate flatness, parallelism, roundness?
class PointMeasure():
    def __init__(self):
        self.P_plane_loaded = False
        self.plane_ready = True
        self.ref_Q = np.array([0,0,0])      # Origin point on plane
        self.ref_S = np.array([1,0,0])      # Vector 1 on plane
        self.ref_T = np.array([0,1,0])      # Vector 2 on plane
        self.ref_n = np.array([0,0,1])      # Normal unit vector
        self.ref_coef = np.array([0,0,1,0]) # Coefficients A B C D, where plane is defined as Ax + By + Cy + D = 0
        self.ref_D = np.array([0])          # Coefficient D (redundant?)
        self.P_plane = []            # point to measure 
        self.intersect = []  # intersection point on the plane
        self.d = []          # scalar distance between point and plane
        self.line_plane = []       # list in plot-able form for drawing measurement line
        self._plane_input_index = 0                      # internal counter to keep track of reference point entry

        self.P_ref = []         # reference for point-to-point 
        self.P_point = []     # point to measure (point to point)
        self.P_point_loaded = False
        self.P_ref_loaded = False
        self.line_point = []
        self._point_input_index = 0     
        self.d_point_point = 0

        self.averaging_window = 24
        self.x = []
        self.y = []
        self.z = []

    def buffer_full(self):
        if(len(self.x) < self.averaging_window):
            return False
        else:
            return True
        
    def clear_point_average(self):
        self.x = []
        self.y = []
        self.z = []
    
    def save_point_average(self, XYZ, override=False):
        if(len(self.x) < self.averaging_window):
            self.x.append(XYZ[0])
            self.y.append(XYZ[1])
            self.z.append(XYZ[2])

        if(len(self.x) >= self.averaging_window or override==True):
            meanx = np.mean(self.x)
            meany = np.mean(self.y)
            meanz = np.mean(self.z)
            sx = np.std(self.x)
            sy = np.std(self.y)
            sz = np.std(self.z)
            print(">Samples: %d" % len(self.x))
            print(">MEAN XYZ: %.4f %.4f %.4f" % (meanx, meany, meanz))
            print(">SDEV XYZ: %.4f %.4f %.4f" % (sx, sy, sz))
            self.save_point([meanx, meany, meanz])
            self.save_point_to_plane([meanx, meany, meanz])

            if(override == True):
                pygame.mixer.Sound.play(bamboo_sound)
            else:
                pygame.mixer.Sound.play(CMM_beep)

    def save_point(self, XYZ):
        if(self._point_input_index == 0):
            self.P_ref = np.array(XYZ)
            self.P_ref_loaded = True
            self.P_point_loaded = False
            self._point_input_index = (self._point_input_index + 1) % 2
            return
        elif(self._point_input_index == 1):
            self.P_point = np.array(XYZ)
            self.P_point_loaded = True
            self._point_input_index = (self._point_input_index +1) % 2
            self.line_point = np.vstack([self.P_point, self.P_ref]).T

            self.deltax = self.P_point[0] - self.P_ref[0]
            self.deltay = self.P_point[1] - self.P_ref[1]
            self.deltaz = self.P_point[2] - self.P_ref[2]
            self.d_point_point = np.sqrt(self.deltax**2 + self.deltay**2 + self.deltaz**2)
            return
    
    def clear_point_to_point(self):
        self.P_ref = []
        self.P_point = []
        self.P_point_loaded = False
        self.P_ref_loaded = False
        self.deltax = 0
        self.deltay = 0
        self.deltaz = 0
        self.d_point_point = 0

    # save a reference point and increment the index to the next one. Order is important. First get Q (origin) then two vectors by right hand rule to orient the normal axis
    def save_ref_plane_point(self, XYZ):
        if(self._plane_input_index == 0):
            # invalidate last measurement if the plane changed
            self.clear_point_to_plane()
            self.plane_ready = False
            self.P_plane_loaded = False
            self.ref_Q = np.array(XYZ)
            print(">Saved ref Q: " + str(self.ref_Q))
            self._plane_input_index = (self._plane_input_index + 1) % 3
            return
        
        elif(self._plane_input_index == 1):
            self.ref_S = np.array(XYZ)
            print(">Saved ref S: " + str(self.ref_S))
            self._plane_input_index = (self._plane_input_index + 1) % 3
            return
        
        elif(self._plane_input_index == 2):
            self.ref_T = np.array(XYZ)
            print(">Saved ref T: " + str(self.ref_T))
            b1 = self.ref_S - self.ref_Q
            print(">b1: " + str(b1))
            b2 = self.ref_T - self.ref_Q
            print(">b2: " + str(b2))
            if(not np.array_equal(b1,b2)):
                N = np.cross(b1,b2)
                print(">N: " + str(N))
                self.ref_n = N/ np.linalg.norm(N)
                self.ref_D = self.ref_n.dot(self.ref_Q.T)
                self.ref_coef = np.hstack([self.ref_n, self.ref_D])
                print(">Plane ready")
                pygame.mixer.Sound.play(CMM_beep)
                print(">[A, B, C, D]: " + str(self.ref_coef))
                print(">Q: " + str(self.ref_Q))
                self.plane_ready = True
            else:
                print(">Basis vectors for reference plane not independent. Start over")
                pygame.mixer.Sound.play(bamboo_sound)
                self.plane_ready = False

            self._plane_input_index = (self._plane_input_index + 1) % 3
        else:
            pass
        
    def save_point_to_plane(self, XYZ):
        self.P_plane_loaded = True
        self.P_plane = np.array(XYZ)
        v = XYZ - self.ref_Q
        self.d = np.dot(v,self.ref_n)
        if(self.plane_ready):
            self.intersect = self.P_plane - self.d*self.ref_n
            self.line_plane = np.vstack([self.P_plane, self.intersect]).T

    def clear_point_to_plane(self):
        self.P_plane_loaded = False
        self.P_plane = []
        self.line_plane = []
        self.ref_Q = np.array([0,0,0])      # Origin point on plane
        self.ref_S = np.array([1,0,0])      # Vector 1 on plane
        self.ref_T = np.array([0,1,0])      # Vector 2 on plane
        self.ref_n = np.array([0,0,1])      # Normal unit vector
        self.ref_coef = np.array([0,0,1,0]) # Coefficients A B C D, where plane is defined as Ax + By + Cy + D = 0
        self.ref_D = np.array([0])          # Coefficient D (redundant?)
        self.plane_ready = True

    def get_point_to_plane_d(self) -> float:
        if(not self.P_plane_loaded):
            return None             # no point loaded, return NaN
        else:
            return self.d
    
    def get_point_to_plane_intersect(self):
        if(not self.P_plane_loaded or not self.plane_ready):
            return None
        else:
            return self.intersect

class PlotData():
    def __init__(self):
        self.path_points = 64                         # max number of points to draw path of tip 
        self.tip_points = 1                           # max number of points to show only the tip location 
        self.csv_points = 2000                        # max number of points in csv point cloud. This is pretty much as high as we can go until I optimize things 
                                                      # It may require PyQT to push this significantly higher.
        self.avg_points = 16                          # We can try to mitigate "noise" from the error propagation and encoder resolution limit by average the last N points 

        self.x = []
        self.y = []
        self.z = []
        self.tipx = []
        self.tipy = []
        self.tipz = []
        self.savex = []
        self.savey = []
        self.savez = []
        self.dirx = []
        self.diry = []
        self.dirz = []

        self.joints = [None]*(NUM_ENCODERS)

    def update_joints(self,joints):
        assert len(joints) == NUM_ENCODERS
        if(len(joints) == NUM_ENCODERS):
            self.joints = joints
    
    def append_xyz(self, dx, dy, dz, dirx, diry, dirz):
        self.x.append(dx)
        self.y.append(dy)
        self.z.append(dz)
        self.tipx.append(dx)
        self.tipy.append(dy)
        self.tipz.append(dz)
        self.dirx.append(dirx)
        self.diry.append(diry)
        self.dirz.append(dirz)

        if len(self.x) > self.path_points:        # remove the oldest point to limit list length
            self.x.pop(0)
            self.y.pop(0)
            self.z.pop(0)

        if len(self.tipx) > self.tip_points:        # remove the oldest point to limit list length
            self.tipx.pop(0)
            self.tipy.pop(0)
            self.tipz.pop(0)
        
        if len(self.dirx) > 1:
            self.dirx.pop(0)
            self.diry.pop(0)
            self.dirz.pop(0)

    def set_num_points(self, num_points):
        self.path_points = num_points

    def clear_csv_data(self):
        self.savex = []
        self.savey = []
        self.savez = []
        
    def clear_display_data(self):
        self.x = []
        self.y = []
        self.z = []
        self.tipx = [0]
        self.tipy = [0]
        self.tipz = [0]
    
    def testCSV(self):
        for i in range(self.csv_points):
            randx = random.randint(-500,500)
            randy = random.randint(-500,500)
            randz = random.randint(-500,500)
            print("Generated random coordinate "+ str(i))
            self.savex.append(randx)
            self.savey.append(randy)
            self.savez.append(randz)
        pass

class Arm():
    def __init__(self, portname = None):
        try:
            self.ser = serial.Serial(portname,115200, timeout=5)       # this should not be hardcoded, obviously
        except:
            pass
        self.portname = portname

    def open(self):
        if(self.ser.is_open):
            self.ser.close()
        print(">Attempting to open serial port...")
        self.ser.open()
        # self.ser.reset_output_buffer()
        # self.ser.reset_input_buffer()
        time.sleep(1)                       # this is apparently necessary to let the interface come up
        print(">Serial port opened")

    def home_arm(self):
        print(">Home arm")
        self.ser.write('h'.encode('utf-8'))
        # self.wait_for_response()
    
    def send_query(self, command : int):
        print(">Sent query: {}".format(command))
        self.ser.write(command.encode('utf-8'))
        self.wait_for_response()

    def send_command(self, command : int):
        print(">Sent command: {}".format(command))
        self.ser.write(command.encode('utf-8'))

    # Connect and wait for handshaking between arduino and microscribe
    # must come after open()
    def wait_for_init(self):
        timeout = 3.0
        time_start = timeit.default_timer()
        initialized = False
        if(self.ser.is_open):
            print(">Waiting for hardware initialization...")
            self.send_command('r')
            while(initialized is False and (timeit.default_timer()-time_start < timeout)):        # add a timeout
                serial_rx= self.ser.readline().decode('utf-8').rstrip('\n')
                if(not serial_rx.isspace()):
                    print(serial_rx)
                if "READY" in serial_rx:
                    print(">Arm initialized")
                    initialized = True
                    return True
            print("Timed out waiting for init")
            return False       

    def wait_for_response(self):
        timeout = 3.0
        time_start = timeit.default_timer()
        print(">Waiting for response")
        while(timeit.default_timer() - time_start < timeout):        # add a timeout
            serial_rx= self.ser.readline().decode('utf-8').rstrip()
            if len(serial_rx)>0:
                print(">Got a response:")
                print(serial_rx)
                return
        print(">Timed out waiting for response")


class  View():
    def __init__(self, arm = None, plotdata = None, point2plane = None):
        # if arm is None:
        #     arm = Arm()
        self.arm = arm          

        if plotdata is None:
            plotdata = PlotData()
        self.data = plotdata

        if point2plane is None:
            point2plane = PointMeasure()
        self.point2plane = point2plane

        plt.style.use('dark_background')
        self.plt = plt
        self.fig = plt.figure(figsize = (10,10), facecolor = 'black')

        # Two ways to generate 3D Axes
        # add_subplot is the proper way to do this. https://github.com/matplotlib/matplotlib/issues/24639
        # add_subplot() or add_axes()
        #self.ax = self.fig.add_subplot(111, projection = "3d")
        self.ax = self.fig.add_axes([0.05, 0.05, 0.9, 0.9], projection='3d', facecolor = 'black')


        self.ax.mouse_init(rotate_btn = 2, pan_btn = 1, zoom_btn=3)                         # disable the drag zoom
        #self.fig.canvas.callbacks._connect_picklable('scroll_event', self._on_scroll)      # alternate method for attaching event callbacks used inside mpl. 
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)
        # self.fig.canvas.mpl_connect('key_press_event', self._on_press)
        # self.fig.canvas.mpl_connect('key_release_event', self._on_release)

        # Non-robot plotting objects. The robot handles its own objects 
        # self.path = self.ax.plot([],[],[], color=styluscolor, alpha = 0.3, animated=True)
        # self.point_graph = self.ax.plot([],[],[], color=tipcolor, alpha = 1,linestyle="",markersize=10, marker = '.', animated=True)
        # self.cloud = self.ax.plot([],[],[], color=cloudcolor, alpha = 1,linestyle="",markersize=5, animated=True)
        #self.plot_objects = []

        font = 'monospace'
        #self.text1 = self.fig.text(0, 0.00, "NUMBER OF POINTS", va='bottom', ha='left',color=indicatorcolor,fontsize=7, name = font)  
        self.textreadout = self.fig.text(0.5,0.94, "XYZ DATA", va='center', ha='center',color=readoutcolor,fontsize=18, name = font)
        self.textreadout.set_bbox(dict(facecolor='black', alpha=1, edgecolor=readoutcolor))
        self.textcloudpoints = self.fig.text(0, 0.015, "NUMBER OF SAVED POINTS", va='bottom', ha='left', color=indicatorcolor, fontsize = 7, name = font)
        self.textfps = self.fig.text(0, 0.03, "FRAME TIME", va='bottom', ha='left', color=indicatorcolor, fontsize = 7, name = font)
        self.textpointplane = self.fig.text(0.5, 0.12, "NORMAL DISTANCE TO PLANE", va='bottom', ha='center', color = P2Plcolor, fontsize = 18, name = font)
        self.textpointplane.set_bbox(dict(facecolor='black', alpha=1, edgecolor=P2Plcolor))
        self.textplanerefpoints = self.fig.text(1, 0, "NUMBER OF PLANE REFERENCE POINTS", va='bottom', ha='right', color = indicatorcolor, name = font, fontsize = 7)
        self.textpointpoint = self.fig.text(0.5, 0.06, "POINT TO POINT", va = 'bottom', ha = 'center', color = P2Pcolor, fontsize = 18, name = font)
        self.textpointpoint.set_bbox(dict(facecolor='black', alpha=1, edgecolor=P2Pcolor))
        self.textpoint2 = self.fig.text(1, 0.015, "POINT 2", va = 'bottom', ha = 'right', color = indicatorcolor, fontsize = 7, name = font)
        self.textpoint1 = self.fig.text(1, 0.03, "POINT 1", va = 'bottom', ha = 'right', color = indicatorcolor, fontsize = 7, name = font)
        self.textdisconnected = self.fig.text(0.5,0.5, "DISCONNECTED", va = 'bottom', ha = 'center', color = 'red', fontsize = 32, name = font)
        self.textdisconnected.set_bbox(dict(facecolor='black', alpha=1, edgecolor='black'))
        
        self.connected = False
        self.showPath = True
        self.showcsv = True
        self.showstylus = True
        self.showp2plane = True
        self.showp2p = True
        self._enable_drag = False

        self.cone_height_res = 10
        self.cone_angle_res = 16
        self.cone_extension_height = 100
        
        self.current_frame_time = timeit.default_timer()
        self.current_frame_avg_queue = [self.current_frame_time]
        self.start = timeit.default_timer()

        link0 = Link(fixed = True, draw_frame = True)                                # origin frame
        link1 = Link(parent = link0, D=210.820007)
        link2 = Link(parent = link1, D=-22.250398, A =24.307800, alpha = 1.567824)
        link3 = Link(parent = link2, D=-0.025400, A = 260.400787, alpha = 0.002684, beta = -0.002780)
        link4 = Link(parent = link3, D=234.848403, A=13.893800, alpha = 1.567920)
        link5 = Link(parent = link4, D=8.128000, A = -10.160000, alpha = -1.572618, draw_link = False)
        link6 = Link(parent = link5, D=-134.010391,A = 10.160000, alpha = -1.569550, draw_joint = False, draw_link = False, draw_frame = False)
        links = [link0,link1,link2,link3,link4,link5,link6]
        self.robot = Robot(links)
        self.robot_render = LinkRender(links, ax = self.ax)

        self.showrobot = True
    
    # Questionable thread safety.... Watch this carefully
    # The idea is try not to mess with variables outside of the local scope here, other than queues.
    # We will have to commandeer the arm serial port, but once the connection is established we do not touch the serial port outside of the worker.
    def serial_worker(self):
        serial_active = False
        tip_packet_cts = True
        current_frame_time = timeit.default_timer()
        current_frame_avg_queue = [self.current_frame_time]
        start = timeit.default_timer()

        while(True):
            try:
                item = serial_worker_in_queue.get(block=False)
                if(item is None): # time to die
                    break
                elif(item is START_WORKER):
                    if(self.arm is not None):
                        print(">Serial worker: running")
                        tip_packet_cts = True
                        serial_active = True
                        serial_worker_out_queue.put(WORKER_RUNNING)
                elif(item is STOP_WORKER):
                    print(">Serial worker: stopped") 
                    serial_active = False
                    serial_worker_out_queue.put(WORKER_STOPPED)
                elif(item is HOME_ARM):
                    print(">Serial worker: home arm")
                    if(self.arm is not None):
                        self.arm.home_arm()
                        tip_packet_cts = True
            except:
                # print('Serial task scheduler: got nothing, waiting a while...')
                pass

            if(serial_active is True):
                #print("Serial worker is polling now")
                try:
                    if(tip_packet_cts == True):
                        self.arm.ser.write(('t>').encode('utf-8'))     # combined command for everything: thetas, then calculated tip position. 
                        tip_packet_cts = False                         # last command was tip position, so we'll be looking for that.
                    
                    while((not tip_packet_cts) and self.arm.ser.in_waiting):         
                        try:
                            serial_rx = self.arm.ser.readline()
                            indata = str(serial_rx[0:len(serial_rx)-2].decode("utf-8"))
                            if(indata.startswith("XYZ")):
                                xyz = indata.split(",")       # Data format for Arduino: X Y Z alpha beta gamma (coords, stylus angles)
                                dx = float(xyz[1])
                                dy = float(xyz[2])
                                dz = float(xyz[3])    
                                dirx = float(xyz[4]) 
                                diry = float(xyz[5])
                                dirz = float(xyz[6])
                                # print(xyz)
                                self.data.append_xyz(dx,dy,dz,dirx,diry,dirz)
                                tip_packet_cts = True
                                
                            elif(indata.startswith("THETA")):
                                enc = indata.split(",")       # Data format: encoder 1,2,3,4,5
                                self.data.update_joints(np.array(enc[1:], dtype=float))
                                # print("joints: " + str(self.data.joints))
                                self.robot.set_angles(self.data.joints, base_offset = True)         

                        except UnicodeDecodeError:
                            print(repr(e))
                            print(">Serial data parse error. Skipping packet")
                            pass
                    
                        except serial.SerialException:     # couldn't read - lost connection
                            self.detach_arm()
                            print(">Read failed. Restart program or scan again")

                except Exception as e:                                         # couldn't write - lost connection
                    self.detach_arm()
                    print(repr(e))
                    print(">Write failed. Restart program or scan again")
                pass
            else:
                pass
            time.sleep(0.0125) # we've got to let other guys access the data

        print(">Exiting serial worker thread")

    def _on_close_fig(self, event):
        self.detach_arm()

    def _disable_pan(self):
        if(self._enable_drag == True):       
            self._enable_drag = False
            self.ax.mouse_init(rotate_btn = 2, pan_btn = 1, zoom_btn=[])     # bind middle mouse to rotate on release. 

    def _enable_pan(self):
        if(self._enable_drag == False):
            self._enable_drag = True
            self.ax.mouse_init(rotate_btn = [], pan_btn = 2, zoom_btn=[])     # bind middle mouse to pan on press. Has a touch of extra lag due to key repeating constantly firing the event and interrupting the thread.
    

    def _on_scroll(self, event):
        w = self.ax._pseudo_w
        h = self.ax._pseudo_h
        sens = 0.1
        if(event.button == 'down'):
            scale = (1 + sens)

        elif(event.button == 'up'):
            scale = (1 - sens)
        self.ax._scale_axis_limits(scale, scale, scale)
        self.fig.canvas.draw_idle()

    def attach_arm(self, arm):
        self.arm = arm
        pass

    def detach_arm(self):
        serial_worker_in_queue.put(STOP_WORKER)
        self.arm = None

    def xy(self):
        self.ax.view_init(elev=90, azim=-90)
        self.update_main_canvas()

    def xz(self):
        self.ax.view_init(elev=0, azim=-90)
        self.update_main_canvas()

    def yz(self):
        self.ax.view_init(elev=0, azim=90)
        self.update_main_canvas()
    
    # # see: https://sabopy.com/py/matplotlib-3d-37/
    # def calculate_cone_const(self, height=15, radius=3, height_ext = 100):
    #     angle_res = self.cone_angle_res
    #     height_res = self.cone_height_res
    #     theta = np.linspace(0, 2*np.pi, angle_res)
    #     r = np.linspace(0, radius, height_res)
    #     t,R =np.meshgrid(theta, r)
    #     X = np.array(R*np.cos(t))
    #     Y = np.array(R*np.sin(t))
    #     Z = np.array(R*height/radius)

    #     Xext = X[height_res-1, :]
    #     Yext = Y[height_res-1, :]
    #     Zext = np.ones(angle_res) * (height+height_ext)

    #     X = np.vstack([X,Xext])
    #     Y = np.vstack([Y,Yext])
    #     Z = np.vstack([Z,Zext])

    #     return (X,Y,Z)

    # def recover_surface(self,XYZ,extension=True):
    #     angle_res = self.cone_angle_res
    #     height_res = self.cone_height_res
    #     if(extension is True):
    #         height_res = height_res + 1
    #     X = np.empty((height_res,angle_res))
    #     Y = np.empty((height_res,angle_res))
    #     Z = np.empty((height_res,angle_res))
    #     for i in range(height_res):
    #         X[i,:] = XYZ[0,i*angle_res:i*angle_res+angle_res]
    #         Y[i,:] = XYZ[1,i*angle_res:i*angle_res+angle_res]
    #         Z[i,:] = XYZ[2,i*angle_res:i*angle_res+angle_res]
    #     return (X,Y,Z)
    
    # # draw cone with given rotation 
    # def draw_cone_euler(self, ax, x,y,z, dirx,diry,dirz, conecolor='None'):
    
    #     # get copy of cone points constants
    #     X = self.coneX.copy()
    #     Y = self.coneY.copy()
    #     Z = self.coneZ.copy()

    #     # execute rotation 
    #     XYZprime = np.stack( [X.ravel(), Y.ravel(), Z.ravel()] , axis = 0)
    #     XYZprime = self.euler_rot(XYZprime, dirx, diry, dirz)

    #     # plot surface
    #     (Xp,Yp,Zp) = self.recover_surface(XYZprime)             # get surface-able vectors from a plain XYZ point cloud 
    #     graph = ax.plot_surface(Xp+x, Yp+y, Zp+z,alpha=0.35,color=conecolor)          
    #     return graph

    # see: https://stackoverflow.com/questions/12341159/creating-a-3d-cone-or-disk-and-keep-updating-its-axis-of-symmetry-with-matplotli 
    def euler_rot(self,XYZ,phi,theta,psi):
        '''Returns the points XYZ rotated by the given euler angles'''

        ERot = np.array([[np.cos(theta)*np.cos(psi), 
                        -np.cos(phi)*np.sin(psi) + np.sin(phi)*np.sin(theta)*np.cos(psi), 
                        np.sin(phi)*np.sin(psi) + np.cos(phi)*np.sin(theta)*np.cos(psi)],
                        [np.cos(theta)*np.sin(psi), 
                        np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(theta)*np.sin(psi),
                        -np.sin(phi)*np.cos(psi) + np.cos(phi)*np.sin(theta)*np.sin(psi)],
                        [-np.sin(theta),
                        np.sin(phi)*np.cos(theta),
                        np.cos(phi)*np.cos(theta)]])

        return ERot.dot(XYZ)
    
    # archive. We don't do this anymore
    def draw_cone_old(self, ax, x,y,z, dirx,diry,dirz):
        # draw a cone with tip at x,y,z and orientation given by dirx,y,z

            # generate X (m*n),Y (m*n), Z(m*n) basic cone
            # see: https://sabopy.com/py/matplotlib-3d-37/
            height = 200
            theta = np.linspace(0, 2*np.pi, 20)
            r = np.linspace(0, 20, 20)
            t,R =np.meshgrid(theta, r)

            X = np.array(R*np.cos(t))
            Y = np.array(R*np.sin(t))
            Z = np.array(R*height/r.max())

            print("Size of Z:")
            print(np.shape(Z))

            # basis vectors
            ihat = [1,0,0]
            jhat = [0,1,0]
            khat = [0,0,1]

            # calculate new rotation matrices
            x_rot_M = self.rotation_matrix(ihat, dirx)
            y_rot_M = self.rotation_matrix(jhat, diry)
            z_rot_M = self.rotation_matrix(khat, dirz)

            Xprime = X.copy()                       # if you don't use a copy on array, the new object will point to the same memory location. 
            Yprime = Y.copy()
            Zprime = Z.copy()

            # slice by slice rotation of the cone
            # clumsy for loops because of the 2D X, Y, Z arrays used to render a good cone. This could be much much better
            for i in range(len(Xprime[:][1])):
                XYZprime = np.stack( [Xprime[i][:], Yprime[i][:], Zprime[i][:]] , axis = 0)
                # print('X:')
                # print(np.shape(X[i][:]))
                # print(X[i][:])
                # print('XYZ concatenated:')
                # print(XYZprime)
                Xprime[i][:] = np.dot(x_rot_M[0][:],XYZprime)
                Yprime[i][:] = np.dot(x_rot_M[1][:],XYZprime)
                Zprime[i][:] = np.dot(x_rot_M[2][:],XYZprime)

            for i in range(len(X[:][1])):
                XYZprime = np.stack( [Xprime[i][:], Yprime[i][:], Zprime[i][:]] , axis = 0)
                Xprime[i][:] = np.dot(y_rot_M[0][:],XYZprime)
                Yprime[i][:] = np.dot(y_rot_M[1][:],XYZprime)
                Zprime[i][:] = np.dot(y_rot_M[2][:],XYZprime)

            for i in range(len(X[:][1])):
                XYZprime = np.stack( [Xprime[i][:], Yprime[i][:], Zprime[i][:]] , axis = 0)
                Xprime[i][:] = np.dot(z_rot_M[0][:],XYZprime)
                Yprime[i][:] = np.dot(z_rot_M[1][:],XYZprime)
                Zprime[i][:] = np.dot(z_rot_M[2][:],XYZprime)

            # print("X: ")
            # print(X.ctypes.data)
            # print("Xprime: ")
            # print(Xprime.ctypes.data)

            graph = ax.plot_surface(Xprime+x, Yprime+y, Zprime+z,alpha=0.35)          
            return graph

    def rotation_matrix(self, axis, theta):
    #General purpose rotation matrix with single angle
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    def draw_ref_plane(self, planecolor = 'plum', alpha =0.2):
        if(self.point2plane.plane_ready):
            # get the bounding limits of the current axes in x y
            curxlim3d = self.ax.get_xlim()
            curylim3d = self.ax.get_ylim()
            curzlim3d = self.ax.get_zlim()
            scale = 1
            xmin = curxlim3d[0]*scale
            xmax = curxlim3d[1]*scale
            ymin = curylim3d[0]*scale
            ymax = curylim3d[1]*scale
            zmin = curzlim3d[0]*scale
            zmax = curzlim3d[1]*scale
        
            threshold = 0.5
            A,B,C,D = self.point2plane.ref_coef
            graph = None

            # three cases, computation of X,Y,Z meshes needs to be different when coefficients near 0
            if(np.abs(C) > threshold):
                X = np.array([xmin, xmin, xmax, xmax])
                Y = np.array([ymin, ymax, ymin, ymax])
                Z = -(A*X + B*Y - D)/C
                #print(Z)
                graph = self.ax.plot_trisurf(X, Y, Z, antialiased = True, color = planecolor, alpha = alpha)
            elif(np.abs(A) > threshold):
                Y = np.array([ymin, ymin, ymax, ymax])
                Z = np.array([zmin, zmax, zmin, zmax])
                X = -(B*Y + C*Z - D)/A
                graph = self.ax.plot_trisurf(X, Y, Z, antialiased = True, color = planecolor, alpha = alpha)
            elif(np.abs(B) > threshold):
                X = np.array([xmin, xmin, xmax, xmax])
                Z = np.array([zmin, zmax, zmin, zmax])
                Y = -(A*X + C*Z - D)/B
                graph = self.ax.plot_trisurf(X, Y, Z, antialiased = True, color = planecolor, alpha = alpha)
            return graph
        else:
            pass
    
            

    def draw_point_point_dist(self,linecolor = 'green'):
        if(self.point2plane.P_ref_loaded and self.point2plane.P_point_loaded):
            line = self.point2plane.line_point.copy()
            graph = self.ax.plot(line[0,:], line[1,:], line[2,:], color = linecolor, alpha=1, linewidth = 1)
            return graph
        else:
            pass

    def draw_point_plane_dist(self,linecolor = 'orange'):
        if(self.point2plane.P_plane_loaded and self.point2plane.plane_ready):
            line = self.point2plane.line_plane.copy()
            graph = self.ax.plot(line[0,:], line[1,:], line[2,:], color = linecolor, alpha=1, linewidth = 1)
            return graph
        else:
            pass

    def remove_plots(self):
        #print("Plot objects list:")
        #print(self.plot_objects)
        for i in range(len(self.plot_objects)):
            plot_object = self.plot_objects[i]

            # print("Deleting link %i index %i" % (link_index, i))
            # print(plot_object)
            if(plot_object is not None):
                # Try to delete surface
                if(type(plot_object) is mpl_toolkits.mplot3d.art3d.Poly3DCollection or type(plot_object) is mpl_toolkits.mplot3d.art3d.Line3DCollection):
                    try:
                        plot_object.remove()          # this works for 3D Surface (poly3Dcollection)
                        # print("Success")
                    except Exception as e:
                        print("couldn't delete 3DCollection")
                        print (repr(e))
                        pass
                
                else:
                    # Try to delete line (unnecessary if using set_3d to update)
                    try: 
                        plot_line_object = plot_object[0]
                        if(type(plot_line_object) is mpl_toolkits.mplot3d.art3d.Line3D):
                            try: 
                                # print("Plot object:")
                                # print(self.link_plots[index][i])
                                plot_line_object.remove()       # this works for 3D plots (3D Lines)
                                # print("Success")
                                continue
                            except Exception as e:
                                print("couldn't delete Line3D")
                                print(repr(e))
                                pass
                    except:
                        pass
    
    def draw_grid_xy(self):
        curxlim3d = self.ax.get_xlim()
        curylim3d = self.ax.get_ylim()
        curzlim3d = self.ax.get_zlim()
        zpos = 0
        if(self.ax.xaxis.get_ticks_position() == 'default'):
            zpos = curzlim3d[1]
        else:
            zpos = curzlim3d[0]
        
        xmin = curxlim3d[0]
        xmax = curxlim3d[1]
        ymin = curylim3d[0]
        ymax = curylim3d[1]

        self.ax.plot([xmin, xmax, xmax, xmin, xmin], [ymin, ymin, ymax, ymax, ymin], [zpos, zpos, zpos, zpos, zpos], color = axiscolor, linewidth = 0.5)       # There's got to a better way to plot these lines.


    def frame_reset(self):
        # running this every time is a very slow way to animate.
        # depending on the orientation hide some labels
        # From the top down, don't show z labels or ticks
        # From the XZ, don't show y labels or ticks
        # From the YZ, don't show x labels or ticks

        # push and pop the current viewport limits
        curxlim3d = self.ax.get_xlim()
        curylim3d = self.ax.get_ylim()
        curzlim3d = self.ax.get_zlim()
        self.ax.cla()
        #self.ax.grid(False)
        #self.ax.set_axis_off()              # Huge CPU usage savings from disabling axes.
        self.ax.set_xlim3d(curxlim3d)
        self.ax.set_ylim3d(curylim3d)
        self.ax.set_zlim3d(curzlim3d) 
        self.ax.set_zticks([])  

        if(self.ax.elev < 70 and self.ax.elev > -70):
            # self.ax.set_zlabel("Z (mm)", color = axiscolor)         # using the clumsy clear/redraw with cla(): ticks are by default visible, and the labels are by default invisible. So set them like this
            pass
        else:
            self.ax.set_zticks([])

        if((np.abs(self.ax.azim) > 20 and np.abs(self.ax.azim) <160) or np.abs(self.ax.elev) > 5):
            # self.ax.set_xlabel("X (mm)", color = axiscolor)
            pass       
        else:
            self.ax.set_xticks([])

        if(np.abs(self.ax.azim) < 70 or np.abs(self.ax.azim) > 110 or np.abs(self.ax.elev) > 5):
            # self.ax.set_ylabel("Y (mm)", color = axiscolor)
            pass
        else:
            self.ax.set_yticks([])
        # self.draw_grid_xy()

    def plot_init(self):
        print(">Init plot")
        self.reset_axis_limits()
        self.ax.tick_params(axis='x', colors=axiscolor)
        self.ax.tick_params(axis='y', colors=axiscolor)
        self.ax.tick_params(axis='z', colors=axiscolor)

        # self.ax.set_xlabel("X (mm)", color ='grey')
        # self.ax.set_ylabel("Y (mm)", color = 'grey')
        # self.ax.set_zlabel("Z (mm)", color ='grey')

        # _axinfo is at risk of deprecation, be careful using this.
        self.ax.yaxis._axinfo["grid"]['linewidth'] = 0.05
        self.ax.xaxis._axinfo["grid"]['linewidth'] = 0.05
        self.ax.zaxis._axinfo["grid"]['linewidth'] = 0.0
        self.ax.xaxis._axinfo["grid"]['color'] = axiscolor
        self.ax.yaxis._axinfo["grid"]['color'] = axiscolor
        self.ax.zaxis._axinfo["grid"]['color'] = axiscolor
        
        # FINALLY, this is how we set the bounding borders (They're the "pane edges") to the colors we want, or delete them. Could not find documentation for this anywhere 
        # Except here: https://github.com/matplotlib/matplotlib/issues/14022#issuecomment-487419062
        # self.ax.xaxis.pane.set_edgecolor(axiscolor)
        # self.ax.yaxis.pane.set_edgecolor(axiscolor)
        # self.ax.zaxis.pane.set_edgecolor(axiscolor)
        # self.ax.xaxis.pane.set_alpha(0)
        # self.ax.yaxis.pane.set_alpha(0)
        # self.ax.zaxis.pane.set_alpha(0)
        self.ax.xaxis.set_pane_color(color = axiscolor, alpha = 0)
        self.ax.yaxis.set_pane_color(color = axiscolor, alpha = 0)
        self.ax.zaxis.set_pane_color(color = axiscolor, alpha = 0)

        self.ax.xaxis.line.set_color(axiscolor)
        self.ax.yaxis.line.set_color(axiscolor)
        self.ax.zaxis.line.set_color(axiscolor)
        self.ax.xaxis.line.set_alpha(1)
        self.ax.yaxis.line.set_alpha(1)
        self.ax.zaxis.line.set_alpha(0)

    def reset_axis_limits(self):
        self.ax.view_init()
        self.ax.set_xlim3d(-500,500)
        self.ax.set_ylim3d(-500,500)
        self.ax.set_zlim3d(-100, 500)
 
        curxlim3d = self.ax.get_xlim()
        curylim3d = self.ax.get_ylim()
        curzlim3d = self.ax.get_zlim()
        self.ax.set_aspect('equal')                                                              # need this to have cubes look right
        #self.ax.set_box_aspect((np.ptp(curxlim3d), np.ptp(curylim3d), np.ptp(curzlim3d)))       # this also works to make cubes look right
    
    def toggle_robot(self):
        self.showrobot = not self.showrobot

    def toggle_path(self):
        self.showPath = not self.showPath

    def toggle_CSV(self):
        self.showcsv = not self.showcsv

    def togglep2plane(self):
        self.showp2plane = not self.showp2plane
    
    def togglep2p(self):
        self.showp2p = not self.showp2p

    def toggle_stylus(self):
        self.showstylus = not self.showstylus

    def update_main_canvas(self, i = 1):
        # see original: https://stackoverflow.com/questions/50342300/animating-3d-scatter-plot-using-python-mplotlib-via-serial-data
        try:
            item = serial_worker_out_queue.get(block = False)
            if(item == WORKER_RUNNING):
                self.connected = True
            elif(item == WORKER_STOPPED):
                self.connected = False
        except:
            pass

        if(self.connected):
            self.textdisconnected.set_visible(False)

            try: 
                def fps():
                    self.current_frame_time = timeit.default_timer() - self.start
                    self.current_frame_avg_queue.append(self.current_frame_time)
                    if len(self.current_frame_avg_queue) > 10:
                        self.current_frame_avg_queue.pop(0)
                    self.start = timeit.default_timer()
                fps()

                # use this block for clearing and redrawing.
                # clearing and redrawing is super SLOW, mind
                self.frame_reset()

                #self.remove_plots()
                #self.plot_objects = []

                # Update all artists and text
                # We really shouldn't redraw the axes this way. But this works and is fast enough to 10000 points.
                if(self.showrobot is True):
                    # Update robot display
                    self.robot_render.draw_links(linkcolor = linkcolor, jointcolor = jointcolor)
                    # print("D-H computed end effector location: " + str(self.robot.get_end_effector_endpoint()))
                if(self.showstylus is True):
                    #self.draw_cone_euler(self.ax, self.data.tipx[0],self.data.tipy[0],self.data.tipz[0], self.data.dirx[0],self.data.diry[0],self.data.dirz[0], conecolor = styluscolor)
                    self.robot_render.draw_stylus(color = styluscolor)

                self.robot_render.draw_base_frame(color = jointcolor)         # always draw base frame.
                #self.ax.plot(self.data.x[-1],self.data.y[-1],self.data.z[-1],color=tipcolor, marker='.', alpha = 1,linestyle="",markersize=2)
                self.ax.plot(self.robot.get_end_effector_endpoint()[0],self.robot.get_end_effector_endpoint()[1],self.robot.get_end_effector_endpoint()[2],color=tipcolor, marker='.', alpha = 1,linestyle="",markersize=2)
                
                if(self.showPath is True):
                    self.ax.plot(self.data.x,self.data.y,self.data.z, color=styluscolor, alpha = 0.2, linewidth=1)
                if(self.showcsv is True):
                    self.ax.plot(self.data.savex, self.data.savey, self.data.savez, linestyle="", color=cloudcolor, marker = '.', alpha=1, markersize=3)

                # The text is rather clumsily rendered.Takes a lot of lines!
                if(self.point2plane.plane_ready):
                    self.draw_ref_plane(planecolor = planecolor)
                    if(self.point2plane.P_plane_loaded and self.showp2plane):
                        self.draw_point_plane_dist(P2Plcolor)
                        self.textpointplane.set_text("Point to plane: {:>7.3f} mm".format(self.point2plane.d))
                    else:
                        self.textpointplane.set_text("")
                    self.textplanerefpoints.set_text("")
                else:
                    self.textpointplane.set_text("")
                    self.textplanerefpoints.set_text("{:d} reference points defined".format(self.point2plane._plane_input_index))

                if(self.point2plane.P_ref_loaded and self.showp2p):
                    self.textpoint1.set_text("1: ({0:7.2f},{1:7.2f},{2:7.2f})".format(self.point2plane.P_ref[0], self.point2plane.P_ref[1], self.point2plane.P_ref[2]))
                    if(self.point2plane.P_point_loaded):
                        self.textpoint2.set_text("2: ({0:7.2f},{1:7.2f},{2:7.2f})".format(self.point2plane.P_point[0], self.point2plane.P_point[1], self.point2plane.P_point[2]))
                        if(self.showp2p):
                            self.draw_point_point_dist(P2Pcolor)
                            self.textpointpoint.set_text("Point to point: {:>7.3f} mm". format(self.point2plane.d_point_point))
                    else:
                        self.textpointpoint.set_text("")
                        self.textpoint2.set_text("")
                else:
                    self.textpoint1.set_text("")
                    self.textpoint2.set_text("")
                    self.textpointpoint.set_text("")
                
                #self.text1.set_text("{:d} points in path buffer".format(len(self.data.x)))  
                self.textreadout.set_text("(X, Y, Z): ({0:8.3f},{1:8.3f},{2:8.3f}) mm \n($\\phi$, $\\theta$, $\\psi$): ({3:8.3f},{4:8.3f},{5:8.3f}) rad ".format(self.data.x[-1],self.data.y[-1],self.data.z[-1], self.data.dirx[-1], self.data.diry[-1], self.data.dirz[-1])) 
                self.textcloudpoints.set_text("{:d} points in point cloud".format(len(self.data.savex)))
                self.textfps.set_text("Frames/second: {:.2f}".format(1/np.mean(self.current_frame_avg_queue)))
           
            except Exception as e:
                #self.detach_arm()
                print(repr(e))
                print(">Waiting for data update...")
                pass
        else:
            self.textdisconnected.set_visible(True)

class GUI():
    def __init__(self, view = None):
        if view is None:
            view = View()
        self.view = view

        self.title = "Microscribe 3D demo"
        #root = tk.Tk() 
        #root.configure(bg = 'black')
        root = tb.Window(themename='cyborg')
        root.title(self.title + " (no device connected)")
        root.minsize(800,800)
       
        # if you need to pass arguments, use command = lambda: function() - note the parantheses
        menubar = Menu(root)
        filemenu = Menu(menubar, tearoff=0)
        menubar.add_cascade(label = 'File', menu=filemenu)
        filemenu.add_command(label = 'Save point cloud',command = lambda:self.save_csv_points())     

        # microscribe options
        msmenu = Menu(menubar, tearoff = 0)
        menubar.add_cascade(label='Microscribe', menu = msmenu)
        msmenu.add_command(label = 'Auto-scan', command = self.auto_scan)
        msmenu.add_command(label = 'Home', command = self.home_arm)
        
        # view options
        viewmenu = Menu(menubar, tearoff = 0)
        menubar.add_cascade(label = 'View', menu = viewmenu)
        viewmenu.add_command(label = 'Reset viewport', command = self.reset_window)
        self.frozen = tk.BooleanVar()
        self.frozen.set(tk.FALSE)
        viewmenu.add_checkbutton(label = 'Freeze (%s)' % Keys.freeze, onvalue = tk.TRUE, offvalue = tk.FALSE, variable = self.frozen, command = self.toggle_render)
        viewmenu.add_checkbutton(label = 'Hide path', command = self.toggle_path)
        viewmenu.add_checkbutton(label = 'Hide CSV data', command = self.toggle_csv)
        viewmenu.add_checkbutton(label = 'Hide orientation', command = self.toggle_stylus)
        viewmenu.add_checkbutton(label = 'Redirect console', command = self.toggle_console)    
        viewmenu.add_checkbutton(label = 'Hide joints (significant performance gain)', command = self.toggle_robot)
        measModes = Menu(viewmenu, tearoff = 0)
        viewmenu.add_cascade(label = 'Show/hide measurement', menu = measModes)
        mode1 = tk.BooleanVar()            # this is supposed to hide measurements by default
        mode2 = tk.BooleanVar()
        # mode1.set(tk.TRUE)                            # I don't understand why the checkbutton doesn't respond to programmed state!! this should enable these by default
        # mode2.set(tk.TRUE)
        measModes.add_checkbutton(label = 'Hide point to plane', variable = mode1,onvalue = tk.TRUE, offvalue = tk.FALSE,  command = self.toggle_p2plane)
        measModes.add_checkbutton(label = 'Hide point to point', variable = mode2, onvalue = tk.TRUE, offvalue = tk.FALSE, command = self.toggle_p2p)
        viewmenu.add_command(label = ("Snap to XY (%s)" % (Keys.xy)), command = self.view.xy)
        viewmenu.add_command(label = ("Snap to XZ (%s)" % (Keys.xz)), command = self.view.xz)
        viewmenu.add_command(label = ("Snap to YZ (%s)" % (Keys.yz)), command = self.view.yz)

        # operations
        ops = Menu(menubar)
        menubar.add_cascade(label = "Operations", menu = ops)
        ops.add_command(label = 'Reset measurement', command = self.reset_meas)
        ops.add_command(label = 'Reset all data', command = self.reset_plot)
        ops.add_command(label = 'Reset CSV data', command = self.reset_csv)
        ops.add_command(label = "Save reference plane point (%s)" % (Keys.plane), command = self.save_ref_plane)
        ops.add_command(label = "Measure point (%s)" % (Keys.point), command = self.save_point_combined  )          
        ops.add_command(label = "Save cloud point (%s)" % (Keys.cloud), command = self.save_cloud_point  )
        ops.add_checkbutton(label = "Enable averaging", command = self.toggle_average)

        self.view.plot_init()
        self.view.reset_axis_limits()
        canvas = FigureCanvasTkAgg(self.view.fig, master=root)
        canvas.get_tk_widget().configure(background = 'black', highlightcolor='lightgrey', highlightbackground='lightgrey')         # needed to remove unsightly border 
        # canvas.mpl_connect('button_press_event', self.view.ax._button_press)
        # canvas.mpl_connect('button_release_event', self.view.ax._button_release)
        # canvas.mpl_connect('motion_notify_event', self.view.ax._on_move)

        plt.rcParams["keymap.quit"] = []           # Tried to bind Q to save the Q reference point but it's already bound to "quit" the mpl figure. Stupid. Disconnect it here
        plt.rcParams["keymap.back"]=[]              # Actually, I hate all of these. Delete all of them
        plt.rcParams["keymap.copy"]=[]
        plt.rcParams["keymap.forward"]=[]
        plt.rcParams["keymap.fullscreen"]=[]
        plt.rcParams["keymap.grid"]=[]
        plt.rcParams["keymap.grid_minor"]=[]
        plt.rcParams["keymap.help"]=[]
        plt.rcParams["keymap.home"]=[]
        plt.rcParams["keymap.pan"]=[]
        plt.rcParams["keymap.quit_all"]=[]
        plt.rcParams["keymap.save"]=[]
        plt.rcParams["keymap.xscale"]=[]
        plt.rcParams["keymap.yscale"]=[]
        plt.rcParams["keymap.zoom"]=[]

        # for k,v in plt.rcParams.items():
        #     if -1 != k.find("keymap"):
        #         #print("plt.rcParams[\"%s\"]=[]"%(k))           
        #         print("plt.rcParams[%s]=%s"%(k,v))

        #self.log_widget = ScrolledText(root, height=4, width=120, bg = 'black', fg = consolecolor, font=('consolas',8), padx = 10, pady = 10)
        self.log_widget = ST(root, height = 8, width = 120, autohide = True, font=('consolas',8))
        self.log_widget.text.configure(bg = 'black', fg = consolecolor)
        #self.log_label = tk.Label( root, text = "CONSOLE OUTPUT:", bg=consolecolor, fg= 'black', font=('consolas', 8, "bold") )
        self.log_label = tb.Label(root, text = "CONSOLE OUTPUT:", background = consolecolor, foreground='black', font=('consolas', 8, "bold"))
        self.log_widget.pack(padx = 6, side = 'bottom', fill = 'both')
        self.log_label.pack(padx = 10, side = 'bottom', anchor='w')
        canvas.get_tk_widget().pack(padx = 10, pady = 10, side='bottom', fill='both', expand=True)
        #canvas._tkcanvas.pack(side='top', fill='both', expand=True)                                    # I don't know why we also have this, I stole this code after all.

        self.logger = PrintLogger(self.log_widget)                                                           # for redirecting the stdout to the logging widget
        self.root = root
        self.canvas = canvas
        self.menu = menubar

        self.update = True
        self.console_visible = False
        self.connected = False
        self.averaging = False
        self.point_average_ready = True
        self.ani = None

        self.update_console()
        
        self.root.config(menu = menubar)
        root.bind("<Key>", self.key_handler)
        root.bind("<KeyRelease>", self.key_release_handler)
                
        def close_window():         
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__
            pygame.quit()
            if(self.ani is not None):
                try:
                    self.ani.pause()
                except Exception as e:
                    print(repr(e))
                    pass
            serial_worker_in_queue.put(None)
            if(self.view.arm is not None):
                self.view.arm.send_command('x')
                self.view.arm.ser.close()
            self.root.quit() # this doesn't seem to normally run and is needed to close properly
            self.root.destroy()

            
        self.root.protocol("WM_DELETE_WINDOW", close_window)

    def update_main_canvas(self, i = 1):
        self.view.update_main_canvas()

    # see : https://stackoverflow.com/a/14224477
    def serial_ports(self):
        """ Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (256 - i) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port, baudrate = 57600)
                result.append(port)
                s.close()
            except (OSError, serial.SerialException):
                pass
        
        return result
    
    # From a list of available COM ports, identifies the first Microscribe Arduino interface. 
    # Assumes only one is connected.
    def find_microscribe(self,listports = ""):
        target = None
        for port in listports:
            s = serial.Serial(port, baudrate = 115200)
            print(">Checking for Microscribe interface device on " + port)
            if(s.is_open): 
                s.close()
            s.open()
            time.sleep(1)                   # if the Arduino just came up, we need to give it some time.
            s.write('i'.encode('utf-8'))
            timeout = 0.5
            time_start = timeit.default_timer()
            print(">Waiting for ID")
            while(timeit.default_timer() - time_start < timeout):   
                self.root.update()
                if(s.in_waiting):
                    try:
                        response = s.readline().decode('utf-8').rstrip()
                        print(">ID:")
                        print(response)
                        if("microscribe" in response.lower()):
                            target = port 
                            print(">Confirmed Microscribe interface on " + port)
                            return target
                    except:
                        pass
            print(">Timed out waiting for response")
            s.close()

        return target

    # Connects to available Microscribe Arduino interface automatically
    # We may eventually change to reading directly from the Microscribe. 
    # Assumes only one is connected. 
    # Returns false if unsuccessful. True if successful
    def auto_scan(self):
        # Search all COM ports for a response
        success = False
        if(self.view.arm is None):
            # pause serial worker if it's goin
            serial_worker_in_queue.put(STOP_WORKER)

            print(">Scanning serial ports:")
            print(self.serial_ports())

            # if something found, initialize it on the microscribe side, then attach it to the plot so the plot can receive data later
            portname = ''
            try:
                portnames = self.serial_ports()
                portname = self.find_microscribe(portnames)
                arm = Arm(portname)
                arm.open()
                result = arm.wait_for_init()
                if(result == True):
                    self.view.attach_arm(arm) 
                    print(">Arm attached successfully")
                    success = True
                else:
                    print(">Microscribe timed out.")
                    success = False
                
                # if arm was attached, then we can start collecting data and plotting. If not, standby with empty plot.
                if(self.view.arm is not None):
                    arm.ser.reset_input_buffer()
                    arm.ser.reset_output_buffer()
                    print(">Setting datastream mode")
                    arm.send_query('q')
                    self.root.title(self.title + " ({})".format(portname))
                    serial_worker_in_queue.put(START_WORKER)
                    
                    # self.view.plt.get_current_fig_manager().set_window_title(self.title+ " ({})".format(portname))
                    # self.view.plt.show()
                    
            except (IndexError, serial.SerialException):
                print(">Auto-scan unsuccessful.")
                success = False
                # exit()
        
        return success


    def home_arm(self):
        serial_worker_in_queue.put(HOME_ARM)
        # self.view.arm.home_arm()
        self.view.point2plane.clear_point_to_plane()
        self.view.point2plane.clear_point_to_point()
        self.view.frame_reset()
        self.view.update_main_canvas()            # update plot
        #self.canvas.draw()               # send updates to tkinder window
        
    def save_csv_points(self):
        defaultname = 'point_cloud_'+time.strftime("%Y-%m-%d %H-%M-%S", time.localtime())+'.csv'
        f = asksaveasfile(initialfile = defaultname)            # read the docs. this returns an opened filewriter
        if(f is not None):
            with f:
                for i in range(len(self.view.data.savex)):
                    row = "%.2f, %.2f, %.2f\n" % (self.view.data.savex[i], self.view.data.savey[i], self.view.data.savez[i])
                    #print("Write row to file:" + row.rstrip('\n'))
                    f.write(row)
                print(">Wrote {0} lines to {1}".format(len(self.view.data.savex), f.name))
        else:
            print(">No file chosen. Returning")
            pass

    # # Start custom render with threading
    # def render_thread(self):
    #     # plot_worker_in_queue.put(START_WORKER)
    #     pass

    # Start built-in animation. don't use at the same time as the custom render loop
    def render_ani(self):
        try: 
            self.ani = animation.FuncAnimation(self.view.fig, self.update_main_canvas, interval=120, frames=1, blit=False)           # frames = 1 performs... Extremely better. 
            self.view.fig.canvas.draw_idle()   
        except:
            pass

    # Start custom render loop with tkinter scheduler
    def render(self):
        if(self.update):
            self.update_main_canvas()                              # update fig,ax
            #self.canvas.draw_idle()                               # send updated fig, ax to tkinder window. otherwise the update will not happen until I drag the canvas
                                                                  # draw_idle() and draw() are options. draw_idle() is faster but draw() seems to behave more nicely with the window
        self._renderjob = self.root.after(50, self.render)        # schedule updates with .after

    def toggle_render(self, keyboard_input = False):
        if(keyboard_input): 
            if(self.frozen.get() == tk.FALSE):
                self.frozen.set(tk.TRUE)
            else:
                self.frozen.set(tk.FALSE)

        if(self.update):
            try:
                self.ani.pause()
            except:
                pass
        else:
            try:
                self.ani.resume()
            except:
                pass
        self.update = not self.update
        # if(self.view.arm_attached):
        #     self.view.arm.ser.reset_input_buffer()
        #     self.view.tip_packet_cts = True

    def toggle_average(self):
        self.averaging = not self.averaging
        print(">Averaging: " + str(self.averaging))

    def toggle_robot(self):
        self.view.toggle_robot()
        self.view.frame_reset()
        self.view.update_main_canvas()
        #self.canvas.draw()

    def toggle_console(self):
        self.console_visible = not self.console_visible
        self.update_console()

    def update_console(self):
        self.log_label.pack_forget()
        self.log_widget.pack_forget()
        self.canvas.get_tk_widget().pack_forget()
        if(not self.console_visible):
            self.canvas.get_tk_widget().pack(padx = 10, pady = 10, side='bottom', fill='both', expand=True)
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__
        else:
            self.log_widget.pack(side = 'bottom', fill = 'both', anchor = 'w')
            self.log_label.pack(padx = 10, side = 'bottom', anchor='w')
            self.canvas.get_tk_widget().pack(padx = 10, pady = 10, side='bottom', fill='both', expand=True)
            sys.stdout = self.logger
            sys.stderr = self.logger

    def toggle_p2p(self):
        self.view.togglep2p()
        self.view.frame_reset()
        self.view.update_main_canvas()            # update plot
        #self.canvas.draw()               # send updates to tkinder window

    def toggle_p2plane(self):
        self.view.togglep2plane()
        self.view.frame_reset()
        self.view.update_main_canvas()            # update plot
        #self.canvas.draw()               # send updates to tkinder window

    def reset_plot(self):
        print(">Reset plot")
        self.view.data.clear_display_data()
        self.view.data.clear_csv_data()
        self.view.frame_reset()
        self.view.update_main_canvas()            # update plot
        #self.canvas.draw()               # send updates to tkinder window
    
    def toggle_path(self):
        self.view.toggle_path()
        self.view.frame_reset()
        self.view.update_main_canvas()            # update fig, ax
        #self.canvas.draw()                  # send updated fig, ax to tkinder window. otherwise the update will not happen until I interact with the window
    
    def toggle_csv(self):
        self.view.toggle_CSV()
        self.view.frame_reset()
        self.view.update_main_canvas()         
        #self.canvas.draw()     

    def toggle_stylus(self):
        self.view.toggle_stylus()
        self.view.frame_reset()
        self.view.update_main_canvas()
        #self.canvas.draw()

    def reset_window(self):
        self.view.reset_axis_limits()
        self.view.update_main_canvas()
        #self.canvas.draw()

    def reset_csv(self):
        print(">Reset point cloud")
        self.view.data.clear_csv_data()
        self.view.frame_reset()
        self.view.update_main_canvas()            
        #self.canvas.draw()              

    def reset_meas(self):
        print(">Reset point to plane measurement")
        self.view.point2plane.clear_point_to_plane()
        print(">Reset point to point measurement")
        self.view.point2plane.clear_point_to_point()
        self.view.frame_reset()
        self.view.update_main_canvas()            
        #self.canvas.draw()              

    def save_cloud_point(self, e=None):
        self.view.data.savex.append(self.view.data.tipx[0])
        self.view.data.savey.append(self.view.data.tipy[0])
        self.view.data.savez.append(self.view.data.tipz[0])
        if(len(self.view.data.savex)>=self.view.data.csv_points):            # This time, remove the latest point to cap the list length. 
            print(">Cloud full")
            self.view.data.savex.pop()
            self.view.data.savey.pop()
            self.view.data.savez.pop()
        else:
            print(">Saved point to csv")

    def save_ref_plane(self):
        if(self.view.showp2plane):
            XYZ = [self.view.data.tipx[0], self.view.data.tipy[0], self.view.data.tipz[0]]
            print(">Saved reference point at " + str(XYZ))
            self.view.point2plane.save_ref_plane_point(XYZ)

    # def save_point_to_point(self):
    #     XYZ = [self.view.data.tipx[0], self.view.data.tipy[0], self.view.data.tipz[0]]
    #     print(">Measure point to point at " + str(XYZ))
    #     self.view.point2plane.save_point(XYZ)

    # def save_point_to_plane(self):
    #     XYZ = [self.view.data.tipx[0], self.view.data.tipy[0], self.view.data.tipz[0]]
    #     print(">Measure point to plane at " + str(XYZ))
    #     self.view.point2plane.save_point_to_plane(XYZ)
    
    def save_point_averaged(self, override = False):
        XYZ = [self.view.data.tipx[0], self.view.data.tipy[0], self.view.data.tipz[0]]
        print(">Append to average " + str(XYZ))
        self.view.point2plane.save_point_average(XYZ, override)

    def save_point_combined(self):
        pygame.mixer.Sound.play(CMM_beep)
        XYZ = [self.view.data.tipx[0], self.view.data.tipy[0], self.view.data.tipz[0]]
        print(">Measurement at " + str(XYZ))
        if(self.averaging == False):
            self.view.point2plane.save_point(XYZ)
            self.view.point2plane.save_point_to_plane(XYZ)

    # TODO: 
    # move the key handler into the view class, this is all canvas stuff anyways. 
    def key_handler(self,e):           
        #print(e.keysym)
        match e.keysym:
            case Keys.plane: 
                self.save_ref_plane()
            case Keys.point:
                if(self.point_average_ready == True):
                    self.point_average_ready = False
                if(self.averaging and self.view.point2plane.buffer_full() == False):
                    self.save_point_averaged()
                elif(self.averaging == False):
                    self.save_point_combined()
            case Keys.xy:
                self.view.xy()
            case Keys.yz:
                self.view.yz()
            case Keys.xz:
                self.view.xz()
            case Keys.pan:
                self.view._enable_pan()
            case Keys.cloud:
                self.save_cloud_point()
            case Keys.freeze:
                self.toggle_render(keyboard_input = True)
            case _:
                pass
    
    def key_release_handler(self, e):
        match e.keysym:
            case Keys.pan:
                self.view._disable_pan()
            case Keys.point:
                if(self.view.point2plane.buffer_full() == False and self.averaging):
                    self.save_point_averaged(override = True)
                    print(">Average with partial buffer")
                self.view.point2plane.clear_point_average()
                self.point_average_ready = True
                pass
            case _:
                pass
    
class App():
    def __init__(self, view = None, gui = None):
        if view is None:
            view = View()
        if gui is None:
            gui = GUI(view)
        self.gui = gui
        self.view = view


if __name__ == "__main__":
    # Here's the idea:
    # On startup, scan the ports. If you find a microscribe, connect it right away (send 'r')
    # If nothing found, load an empty plot. The GUI owns the plot and can attach an arm to it once connected
    app = App()

    serial_poll_worker = Thread(target = app.view.serial_worker, args = (), daemon = True)
    serial_poll_worker.start()

    #app.view.data.testCSV()
    app.gui.auto_scan()
    app.gui.render_ani()        # use built-in animation scheduler. Do not run this multiple times, it will double schedule the animation updates.
    #app.gui.render()
    app.gui.root.mainloop()

    serial_worker_in_queue.put(None)       # Just in case, shut down the threads
    serial_poll_worker.join()

  
    
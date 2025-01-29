# TODO: Separate the Arm Serial control from the View class so we can handle it better. Possibly in a thread writing to a queue
# TODO: Issue: Increasing display framerate either in render() or with animation() swamps the single app thread. Move animation to its own thread perhaps.
# TODO: Panning the plot is usable in Axes3D by default. Zooming with the right click sucks and should be changed to scroll.
# TODO: Use pyQT for a better UI and built in threading
# TODO: Create proper unit tests for functions

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import griddata
import math
import random

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D

import tkinter as tk
from tkinter import *
from tkinter.filedialog import asksaveasfile

import time
import timeit
import serial


# see: https://stackoverflow.com/questions/13685386/how-to-set-the-equal-aspect-ratio-for-all-axes-x-y-z
from matplotlib import cm
def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def testCSVplot(plot):
        for i in range(plot.csv_points):
            randx = random.randint(-500,500)
            randy = random.randint(-500,500)
            randz = random.randint(-500,500)
            print("Generated random coordinate "+ str(i))
            plot.savex.append(randx)
            plot.savey.append(randy)
            plot.savez.append(randz)

# implementing point to reference plane
# TODO later: evaluate flatness, parallelism, roundness?
class PointToPlane():
    def __init__(self):
        self.point_to_plane_loaded = False
        self.plane_ready = True
        self.ref_Q = np.array([0,0,0])
        self.ref_S = np.array([1,0,0])
        self.ref_T = np.array([0,1,0])
        self.ref_n = np.array([0,0,1])
        self.ref_coef = np.array([0,0,1,0])
        self.ref_D = np.array([0])
        self.point_to_plane = []
        self.point_to_plane_intersect = []
        self.point_to_plane_d = []
        self.point_to_plane_line = []
        self.index = 0
        pass

    # save a reference point and increment the index to the next one. Order is important. First get Q (origin) then two vectors by right hand rule to orient the normal axis
    def save_ref_plane_point(self, XYZ):
        if(self.index == 0):
            self.plane_ready = False
            self.point_to_plane_loaded = False
            self.ref_Q = np.array(XYZ)
            print("Saved ref Q")
            print(self.ref_Q)
            self.index = (self.index + 1) % 3
            return
        
        elif(self.index == 1):
            self.ref_S = np.array(XYZ)
            print("Saved ref S")
            print(self.ref_S)
            self.index = (self.index + 1) % 3
            return
        
        elif(self.index == 2):
            self.ref_T = np.array(XYZ)
            print("Saved ref T")
            print(self.ref_T)
            b1 = self.ref_S - self.ref_Q
            print("b1")
            print(b1)
            b2 = self.ref_T - self.ref_Q
            print("b2")
            print(b2)
            if(not np.array_equal(b1,b2)):
                N = np.cross(b1,b2)
                print("N")
                print(N)
                self.ref_n = N/ np.linalg.norm(N)
                self.ref_D = self.ref_n.dot(self.ref_Q.T)
                self.ref_coef = np.hstack([self.ref_n, self.ref_D])
                print("Calculated norm")
                print("Plane ready")
                print("[A, B, C, D]:")
                print(self.ref_coef)
                print("Q: ")
                print(self.ref_Q)
                self.plane_ready = True
            else:
                print("Basis vectors for reference plane not independent. Start over")
                self.plane_ready = False

            self.index = (self.index + 1) % 3
            return
        else:
            pass
        
    def save_point_to_plane(self, XYZ):
        self.point_to_plane_loaded = True
        self.point_to_plane = np.array(XYZ)
        v = XYZ - self.ref_Q
        self.point_to_plane_d = np.dot(v,self.ref_n)
        if(self.plane_ready):
            self.point_to_plane_intersect = self.point_to_plane - self.point_to_plane_d*self.ref_n
        pass

    def clear_point_to_plane(self):
        self.point_to_plane_loaded = False
        self.point_to_plane = []

    def get_point_to_plane_d(self) -> float:
        if(not self.point_to_plane_loaded):
            return None             # no point loaded, return NaN
        else:
            return self.point_to_plane_d
    
    def get_point_to_plane_intersect(self):
        if(not self.point_to_plane_loaded or not self.plane_ready):
            return None
        else:
            return self.point_to_plane_intersect

class plotData():
    def __init__(self):
        self.path_points = 64                         # max number of points to draw path of tip (ax.plot). 
        self.tip_points = 1                           # max number of points to show only the tip location (ax.scatter)
        self.csv_points = 10000                        # max number of points in csv point cloud. This is pretty much as high as we can go until I optimize things 
                                                      # It may require PyQT to push this significantly higher.
        self.x = []
        self.y = []
        self.z = []
        self.tipx = []
        self.tipy = []
        self.tipz = []
        self.savex = []
        self.savey = []
        self.savez = []
        self.dirx = 0
        self.diry = 0
        self.dirz = 0

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
    
    def calculate_base_location(self):
        # use stylus dir information to calculate endpoint for line showing orientation of stylus
        # calculate from fixed-frame XYZ rotation data
        stylus_length = 200
        pass


class Arm():
    def __init__(self):
        self.ser = serial.Serial('COM8',57600, timeout=5)
        self.initialized = False

    def open_source(self):
        if(not self.ser.is_open):
            self.ser.open()
            self.ser.flushInput()
            time.sleep(2)

    def home_arm(self):
        print(">Home arm")
        self.ser.write('h'.encode('utf-8'))

    # wait for handshaking at bootup
    def wait_for_init(self):
        print(">Waiting for hardware initialization...")
        while(self.initialized is False):        # add a timeout
            serial_rx= self.ser.readline().decode('utf-8').rstrip()
            print(serial_rx)
            if "READY" in serial_rx:
                print(">Arm initialized")
                self.initialized = True
                break

    def wait_for_response(self):
        print(">Waiting for response")
        while(True):        # add a timeout
            serial_rx= self.ser.readline().decode('utf-8').rstrip()
            if len(serial_rx)>0:
                print(">Got a response:")
                print(serial_rx)
                break

class View():
    def __init__(self, arm = None, plotdata = None, point2plane = None):
        if arm is None:
            arm = Arm()
        self.arm = arm          # TODO: get the Arm out of the View class, View should just act on the data. Use semaphores to write/read the lists

        if plotdata is None:
            plotdata = plotData()
        self.data = plotdata

        if point2plane is None:
            point2plane = PointToPlane()
        self.point2plane = point2plane

        plt.style.use('dark_background')

        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection = "3d")

        self.path = self.ax.plot([],[],[], color='aquamarine', alpha = 0.3, animated=True)
        self.point_graph, = self.ax.plot([],[],[], color='turquoise', alpha = 1,linestyle="",markersize=10, marker = '.', animated=True)
        self.cloud = self.ax.plot([],[],[], color='red', alpha = 1,linestyle="",markersize=5, animated=True)
        self.stylus = self.ax.plot([],[],[], color='red', alpha = 0.3,animated=True)

        self.text1 = self.fig.text(0, 0.00, "NUMBER OF POINTS", va='bottom', ha='left',color='lightgrey',fontsize=12)  
        self.text2 = self.fig.text(0.5,0.97, "XYZ DATA", va='top', ha='center',color='lightgrey',fontsize=24)
        self.text3 = self.fig.text(0, 0.03, "NUMBER OF SAVED POINTS", va='bottom', ha='left', color='lightgrey', fontsize = 12)
        self.text4 = self.fig.text(0.5,0.87, "TIP ORIENTATION", va='bottom', ha='center', color='lightgrey', fontsize = 24)
        self.text5 = self.fig.text(0, 0.06, "FRAME TIME", va='bottom', ha='left', color='lightgrey', fontsize = 12)
        self.text6 = self.fig.text(0.5, 0.09, "NORMAL DISTANCE TO PLANE", va='bottom', ha='center', color = 'orange', fontsize = 18)
        
        self.showPath = True
        self.showcsv = True
        self.showstylus = True
        self.showp2plane = True

        self.cone_height_res = 10
        self.cone_angle_res = 10
        self.cone_extension_height = 100
        self.coneX, self.coneY, self.coneZ = self.calculate_cone_const()
        
        self.current_frame_time = timeit.default_timer()
        self.current_frame_avg_queue = [self.current_frame_time]
        self.start = timeit.default_timer()

    
    # see: https://sabopy.com/py/matplotlib-3d-37/
    def calculate_cone_const(self, height=20, radius=3, height_ext = 100):
        angle_res = self.cone_angle_res
        height_res = self.cone_height_res
        theta = np.linspace(0, 2*np.pi, angle_res)
        r = np.linspace(0, radius, height_res)
        t,R =np.meshgrid(theta, r)
        X = np.array(R*np.cos(t))
        Y = np.array(R*np.sin(t))
        Z = np.array(R*height/radius)

        Xext = X[height_res-1, :]
        Yext = Y[height_res-1, :]
        Zext = np.ones(angle_res) * (height+height_ext)

        X = np.vstack([X,Xext])
        Y = np.vstack([Y,Yext])
        Z = np.vstack([Z,Zext])

        return (X,Y,Z)

    def recover_surface(self,XYZ,extension=True):
        angle_res = self.cone_angle_res
        height_res = self.cone_height_res
        if(extension is True):
            height_res = height_res + 1
        X = np.empty((height_res,angle_res))
        Y = np.empty((height_res,angle_res))
        Z = np.empty((height_res,angle_res))
        for i in range(height_res):
            X[i,:] = XYZ[0,i*angle_res:i*angle_res+angle_res]
            Y[i,:] = XYZ[1,i*angle_res:i*angle_res+angle_res]
            Z[i,:] = XYZ[2,i*angle_res:i*angle_res+angle_res]
        return (X,Y,Z)
    
    # draw cone with given rotation 
    def draw_cone_euler(self, ax, x,y,z, dirx,diry,dirz):
    
        # get copy of cone points constants
        X = self.coneX.copy()
        Y = self.coneY.copy()
        Z = self.coneZ.copy()

        # execute rotation 
        XYZprime = np.stack( [X.ravel(), Y.ravel(), Z.ravel()] , axis = 0)
        XYZprime = self.euler_rot(XYZprime, dirx, diry, dirz)

        # plot surface
        (Xp,Yp,Zp) = self.recover_surface(XYZprime)             # get surface-able vectors from a plain XYZ point cloud 
        graph = ax.plot_surface(Xp+x, Yp+y, Zp+z,alpha=0.35,color='cyan')          
        return graph

    # see:https://stackoverflow.com/questions/12341159/creating-a-3d-cone-or-disk-and-keep-updating-its-axis-of-symmetry-with-matplotli 
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
    
    # archive
    # def draw_cone_old(self, ax, x,y,z, dirx,diry,dirz):
    #     # draw a cone with tip at x,y,z and orientation given by dirx,y,z

    #         # generate X (m*n),Y (m*n), Z(m*n) basic cone
    #         # see: https://sabopy.com/py/matplotlib-3d-37/
    #         height = 200
    #         theta = np.linspace(0, 2*np.pi, 20)
    #         r = np.linspace(0, 20, 20)
    #         t,R =np.meshgrid(theta, r)

    #         X = np.array(R*np.cos(t))
    #         Y = np.array(R*np.sin(t))
    #         Z = np.array(R*height/r.max())

    #         print("Size of Z:")
    #         print(np.shape(Z))

    #         # basis vectors
    #         ihat = [1,0,0]
    #         jhat = [0,1,0]
    #         khat = [0,0,1]

    #         # calculate new rotation matrices
    #         x_rot_M = self.rotation_matrix(ihat, dirx)
    #         y_rot_M = self.rotation_matrix(jhat, diry)
    #         z_rot_M = self.rotation_matrix(khat, dirz)

    #         Xprime = X.copy()                       # if you don't use a copy on array, the new object will point to the same memory location. 
    #         Yprime = Y.copy()
    #         Zprime = Z.copy()

    #         # slice by slice rotation of the cone
    #         # clumsy for loops because of the 2D X, Y, Z arrays used to render a good cone. This could be much much better
    #         for i in range(len(Xprime[:][1])):
    #             XYZprime = np.stack( [Xprime[i][:], Yprime[i][:], Zprime[i][:]] , axis = 0)
    #             # print('X:')
    #             # print(np.shape(X[i][:]))
    #             # print(X[i][:])
    #             # print('XYZ concatenated:')
    #             # print(XYZprime)
    #             Xprime[i][:] = np.dot(x_rot_M[0][:],XYZprime)
    #             Yprime[i][:] = np.dot(x_rot_M[1][:],XYZprime)
    #             Zprime[i][:] = np.dot(x_rot_M[2][:],XYZprime)

    #         for i in range(len(X[:][1])):
    #             XYZprime = np.stack( [Xprime[i][:], Yprime[i][:], Zprime[i][:]] , axis = 0)
    #             Xprime[i][:] = np.dot(y_rot_M[0][:],XYZprime)
    #             Yprime[i][:] = np.dot(y_rot_M[1][:],XYZprime)
    #             Zprime[i][:] = np.dot(y_rot_M[2][:],XYZprime)

    #         for i in range(len(X[:][1])):
    #             XYZprime = np.stack( [Xprime[i][:], Yprime[i][:], Zprime[i][:]] , axis = 0)
    #             Xprime[i][:] = np.dot(z_rot_M[0][:],XYZprime)
    #             Yprime[i][:] = np.dot(z_rot_M[1][:],XYZprime)
    #             Zprime[i][:] = np.dot(z_rot_M[2][:],XYZprime)

    #         # print("X: ")
    #         # print(X.ctypes.data)
    #         # print("Xprime: ")
    #         # print(Xprime.ctypes.data)

    #         graph = ax.plot_surface(Xprime+x, Yprime+y, Zprime+z,alpha=0.35, cmap=cm.GnBu)          
    #         return graph
    # def rotation_matrix(self, axis, theta):
    # General purpose rotation matrix with single angle
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    def draw_ref_plane(self):
        if(self.point2plane.plane_ready):
            # get the bounding limits of the current axes in x y
            curxlim3d = self.ax.get_xlim()
            curylim3d = self.ax.get_ylim()

            xmin = curxlim3d[0]
            xmax = curxlim3d[1]
            ymin = curylim3d[0]
            ymax = curylim3d[1]

            X = np.array([xmin, xmin, xmax, xmax])
            Y = np.array([ymin, ymax, ymin, ymax])
            A,B,C,D = self.point2plane.ref_coef
            Z = -(A*X + B*Y - D)/C
            #print(Z)

            graph = self.ax.plot_trisurf(X, Y, Z, antialiased = True, color = 'lavender', alpha = 0.1)
            return graph
        else:
            pass
    
    def draw_point_plane_dist(self):
        if(self.point2plane.point_to_plane_loaded and self.point2plane.plane_ready):
            line = np.vstack([self.point2plane.point_to_plane, self.point2plane.point_to_plane_intersect]).T
            # print(line)
            graph = self.ax.plot(line[0,:], line[1,:], line[2,:], color = 'orange', alpha=1)
            return graph
        else:
            pass

    def draw_p2plane(self):
        pass

        pass
    def frame_reset(self):
        # this is a very slow way to animate. But. It is fine.

        # push and pop the current viewport limits
        curxlim3d = self.ax.get_xlim()
        curylim3d = self.ax.get_ylim()
        curzlim3d = self.ax.get_zlim()
        self.ax.cla()
        self.ax.set_xlim3d(curxlim3d)
        self.ax.set_ylim3d(curylim3d)
        self.ax.set_zlim3d(curzlim3d)
        self.ax.set_xlabel("X (mm)", color ='grey')
        self.ax.set_ylabel("Y (mm)", color = 'grey')
        self.ax.set_zlabel("Z (mm)", color ='grey')

    def init_plot(self):
        print(">Init plot")

        # push and pop the current viewport limits
        curxlim3d = self.ax.get_xlim()
        curylim3d = self.ax.get_ylim()
        curzlim3d = self.ax.get_zlim()
        self.ax.cla()
        self.ax.set_xlim3d(curxlim3d)
        self.ax.set_ylim3d(curylim3d)
        self.ax.set_zlim3d(curzlim3d)
        
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
    
    def reset_axis_limits(self):
        self.ax.set_xlim3d(-500,500)
        self.ax.set_ylim3d(-500,500)
        self.ax.set_zlim3d(-50, 300)
        set_axes_equal(self.ax)
    
    def clear_display(self):
        pass

    def clear_csv_display(self):
        pass

    def togglePath(self):
        self.showPath = not self.showPath

    def toggleCSV(self):
        self.showcsv = not self.showcsv

    def togglemeasure(self):
        self.showp2plane = not self.showp2plane

    def togglestylus(self):
        self.showstylus = not self.showstylus

    def update_lines(self, i=1, path=True, csv=True):
        # see original: https://stackoverflow.com/questions/50342300/animating-3d-scatter-plot-using-python-mplotlib-via-serial-data

        self.current_frame_time = timeit.default_timer() - self.start
        self.current_frame_avg_queue.append(self.current_frame_time)
        if len(self.current_frame_avg_queue) > 10:
            self.current_frame_avg_queue.pop(0)
        self.start = timeit.default_timer()

        # use this block for clearing and redrawing.
        # clearing and redrawing is super SLOW, mind
        self.frame_reset()

        packet_received = False
        self.arm.ser.write(('>').encode('utf-8'))
        while(not packet_received):
            try:
                serial_rx = self.arm.ser.readline()
                try:
                    data = str(serial_rx[0:len(serial_rx)-2].decode("utf-8"))
                    if(data.startswith("XYZ")):
                        xyz = data.split(",")       # Data format for Arduino: X Y Z alpha beta gamma (coords, stylus angles)
                        #print(xyz)
                        dx = float(xyz[1])
                        dy = float(xyz[2])
                        dz = float(xyz[3])    
                        dirx = float(xyz[4]) 
                        diry = float(xyz[5])
                        dirz = float(xyz[6])
                        packet_received = True

                        self.data.x.append(dx)
                        self.data.y.append(dy)
                        self.data.z.append(dz)
                        self.data.tipx.append(dx)
                        self.data.tipy.append(dy)
                        self.data.tipz.append(dz)
                        self.data.dirx = dirx*np.pi/180
                        self.data.diry = diry*np.pi/180
                        self.data.dirz = dirz*np.pi/180

                        if len(self.data.x) > self.data.path_points:        # remove the oldest point to limit list length
                            self.data.x.pop(0)
                            self.data.y.pop(0)
                            self.data.z.pop(0)

                        if len(self.data.tipx) > self.data.tip_points:        # remove the oldest point to limit list length
                            self.data.tipx.pop(0)
                            self.data.tipy.pop(0)
                            self.data.tipz.pop(0)

                        # We really shouldn't redraw the axes this way. But this works
                        self.tip = self.ax.plot(dx,dy,dz,color='turquoise', marker='.', alpha = 1,linestyle="",markersize=5)
                        self.plane = self.draw_ref_plane()
                        if(self.point2plane.plane_ready and self.point2plane.point_to_plane_loaded and self.showp2plane):
                            self.draw_point_plane_dist()
                            self.text6.set_text("Normal distance: {:.3f} mm".format(self.point2plane.point_to_plane_d))
                        else:
                            self.text6.set_text("")
                        if(self.showPath is True):
                            self.path = self.ax.plot(self.data.x,self.data.y,self.data.z, color='aquamarine', alpha = 0.3)
                        if(self.showcsv is True):
                            self.cloud = self.ax.plot(self.data.savex, self.data.savey, self.data.savez, linestyle="", color='red', marker = '.', alpha=1, markersize=5)
                        if(self.showstylus is True):
                            self.stylus = self.draw_cone_euler(self.ax, self.data.tipx,self.data.tipy,self.data.tipz, self.data.dirx,self.data.diry,self.data.dirz)

                        self.text1.set_text("{:d} points in path buffer".format(len(self.data.x)))  
                        self.text2.set_text("(X,Y,Z): ({:.2f}, {:.2f}, {:.2f}) mm".format(dx, dy, dz)) 
                        self.text3.set_text("{:d} points in point cloud".format(len(self.data.savex)))
                        self.text4.set_text("(Roll, Pitch, Yaw): ({:.2f}, {:.2f}, {:.2f}) deg".format(dirx, diry, dirz)) 
                        self.text5.set_text("Frames/second: {:.2f}".format(1/np.mean(self.current_frame_avg_queue)))

                    else:
                        pass
                except UnicodeDecodeError:
                    pass
            except serial.SerialException as e:
            #There is no new data from serial port
                break

class GUI():
    def __init__(self, view = None, arm = None):

        if view is None:
            view = View()
        self.view = view

        root = tk.Tk()
        root.title("Microscribe 3D demo")
        root.minsize(700,700)
        
        self.view.init_plot()
        self.view.reset_axis_limits()

        menubar = Menu(root)
        filemenu = Menu(menubar, tearoff=0)
        menubar.add_cascade(label = 'File', menu=filemenu)
        filemenu.add_command(label = 'Save point cloud',command = self.save_points)
        # microscribe options
        msmenu = Menu(menubar, tearoff = 0)
        menubar.add_cascade(label='Microscribe options', menu = msmenu)
        msmenu.add_command(label = 'Home', command = self.view.arm.home_arm)
        # view options
        view = Menu(menubar, tearoff = 0)
        menubar.add_cascade(label = 'View options', menu = view)
        view.add_command(label = 'Reset all data', command = self.reset_plot)
        view.add_command(label = 'Reset CSV data', command = self.reset_csv)
        view.add_command(label = 'Reset viewport', command = self.reset_window)
        view.add_checkbutton(label = 'Freeze', command = self.toggle_render)
        view.add_checkbutton(label = 'Hide path', command = self.toggle_path)
        view.add_checkbutton(label = 'Hide CSV data', command = self.toggle_csv)
        view.add_checkbutton(label = 'Hide orientation', command = self.toggle_stylus)
        view.add_checkbutton(label = 'Hide measurement', command = self.toggle_measure)

        canvas = FigureCanvasTkAgg(self.view.fig, master=root)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        canvas.mpl_connect('button_press_event', self.view.ax._button_press)
        canvas.mpl_connect('button_release_event', self.view.ax._button_release)
        canvas.mpl_connect('motion_notify_event', self.view.ax._on_move)

        plt.rcParams["keymap.quit"] = ["ctrl+w", "cmd+w"]           # Tried to bind Q to save the Q reference point but it's already bound to "quit" the mpl figure. Stupid. Disconnect it here
    
        self.root = root
        self.canvas = canvas
        self.update = True
                
        def close_window():
            self.root.quit() # this doesn't seem to normally run and is needed to close properly
            self.root.destroy()
            return
        self.root.protocol("WM_DELETE_WINDOW", close_window)

        self.root.config(menu = menubar)
        root.bind("<space>", self.save_cloud_point)
        root.bind("<Key>", self.key_handler)

        self.ani = animation.FuncAnimation(self.view.fig, self.view.update_lines, interval=50, frames=1, blit=False)    
        self.ani.pause()

    def  save_points(self):
        defaultname = 'point_cloud_'+time.strftime("%Y-%m-%d %H-%M-%S", time.localtime())+'.csv'
        f = asksaveasfile(initialfile = defaultname)            # read the docs. this returns an opened filewriter
        with f:
            for i in range(len(self.view.data.savex)):
                row = "%.2f, %.2f, %.2f\n" % (self.view.data.savex[i], self.view.data.savey[i], self.view.data.savez[i])
                print("Write row to file:" + row)
                f.write(row)

    # Start built-in animation. don't use at the same time as the custom render loop
    def render_ani(self):
        try:
            self.ani.resume()
        except:
            pass

    # Start custom render loop with tkinter scheduler
    def render(self):
        if(self.update):
            self.view.update_lines()                              # update fig,ax
            self.canvas.draw()                                    # send updated fig, ax to tkinder window. otherwise the update will not happen until I drag the canvas
                                                                  # draw() and draw_idle() are options. draw_idle() is faster but draw() seems to behave more nicely with the window
        self._renderjob = self.root.after(50, self.render)        # schedule updates with .after
    
    def toggle_render(self):
        
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
        self.view.arm.ser.reset_input_buffer()

    def toggle_measure(self):
        self.view.togglemeasure()

    def reset_plot(self):
        print(">Reset plot")
        self.view.data.clear_display_data()
        self.view.data.clear_csv_data()
        self.view.init_plot()
        self.view.update_lines()            # update plot
        self.canvas.draw()               # send updates to tkinder window
    
    def toggle_path(self):
        self.view.togglePath()
        self.view.init_plot()
        self.view.update_lines()            # update fig, ax
        self.canvas.draw()                  # send updated fig, ax to tkinder window. otherwise the update will not happen until I interact with the window
    
    def toggle_csv(self):
        self.view.toggleCSV()
        self.view.init_plot()
        self.view.update_lines()         
        self.canvas.draw()     

    def toggle_stylus(self):
        self.view.togglestylus()
        self.view.init_plot()
        self.view.update_lines()
        self.canvas.draw()

    def reset_window(self):
        self.view.reset_axis_limits()
        self.view.update_lines()
        self.canvas.draw()

    def reset_csv(self):
        print(">Reset point cloud")
        self.view.data.clear_csv_data()
        self.view.init_plot()
        self.view.update_lines()            
        self.canvas.draw()              

    def save_cloud_point(self, e=None):
        #print(">Saved point to csv")
        self.view.data.savex.append(self.view.data.tipx[0])
        self.view.data.savey.append(self.view.data.tipy[0])
        self.view.data.savez.append(self.view.data.tipz[0])
        if(len(self.view.data.savex)>=self.view.data.csv_points):            # This time, remove the latest point to cap the list length. 
            self.view.data.savex.pop()
            self.view.data.savey.pop()
            self.view.data.savez.pop()

    def save_ref_plane(self):
        print("Save ref point")
        XYZ = [self.view.data.tipx[0], self.view.data.tipy[0], self.view.data.tipz[0]]
        self.view.point2plane.save_ref_plane_point(XYZ)
        pass

    def save_point_to_plane(self):
        print("Save measurement point")
        XYZ = [self.view.data.tipx[0], self.view.data.tipy[0], self.view.data.tipz[0]]
        self.view.point2plane.save_point_to_plane(XYZ)

    def key_handler(self,e=None):
        match e.keysym:
            case 'q': 
                self.save_ref_plane()
            case 'p':
                self.save_point_to_plane()
            case _:
                pass
    
class App():
    def __init__(self, arm = None, view = None, gui = None):
        if arm is None:
            arm = Arm()
        if view is None:
            view = View(arm)
        if gui is None:
            gui = GUI(view)

        self.gui = gui
        self.view = view

if __name__ == "__main__":
    app = App()

    if(not app.view.arm.ser.is_open):
        print(">Opening serial port")
        app.view.arm.ser.open()
        app.view.arm.ser.flushInput()
        time.sleep(2)

    app.view.arm.wait_for_init()
    print(">Setting datastream mode")
    app.view.arm.ser.write('q'.encode('utf-8'))       # turn on query-based reporting
    app.view.arm.wait_for_response()

    #app.gui.render()           # use custom render loop
    app.gui.render_ani()        # use built-in animation scheduler. 

    #testCSVplot(app.view.data)
    app.gui.root.mainloop()

# TODO: mate you gotta define some classes
# template: https://github.com/precise-simulation/mesh-viewer/blob/master/meshviewer_mpl_tk.py#L296-L298
# App class  
# View class        (render elements)
# Controller class  (GUI elements)

# TODO: Separate the Arm Serial control from the View class so we can handle it better

import numpy as np
from scipy.spatial.transform import Rotation as R
import math

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

class plotData():
    def __init__(self):
        self.path_points = 64                         # number of points to draw path of tip (ax.plot). 
        self.tip_points = 1                           # number of points to show only the tip location (ax.scatter)
        self.csv_points = 2048                        # number of points in csv point cloud
        self.x = []
        self.y = []
        self.z = []
        self.tipx = 0
        self.tipy = 0
        self.tipz = 0
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
        self.tipx = 0
        self.tipy = 0
        self.tipz = 0
    
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
            self.ser.write('c'.encode('utf-8'))      # turn on continuous reporting 
            #self.ser.write('q'.encode('utf-8'))       # turn on query-based reporting

    def home_arm(self):
        print(">Home arm")
        self.ser.write('h'.encode('utf-8'))

    # wait for handshaking at bootup
    def wait_for_init(self):
        print(">Waiting for hardware initialization...")
        while(True):        # add a timeout
            serial_rx= self.ser.readline().decode('utf-8')
            print(serial_rx)
            if "READY" in serial_rx:
                print(">Arm initialized")
                break

    def wait_for_response(self):
        print(">Waiting for response")
        while(True):        # add a timeout
            serial_rx= self.ser.readline().decode('utf-8')
            if len(serial_rx)>0:
                print(">Got a response:")
                print(serial_rx)
                break

class View():
    def __init__(self, arm = None, plotdata = None):
        if arm is None:
            arm = Arm()
        self.arm = arm          # TODO: get the Arm out of the View class, View should just act on the data. Use semaphores to write/read the lists

        if plotdata is None:
            plotdata = plotData()
        self.data = plotdata

        self.plot = plt
        self.plot.style.use('dark_background')

        self.fig = self.plot.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection = "3d")

        self.path = self.ax.scatter([],[],[], color='aquamarine', alpha = 0.3)
        self.tip = self.ax.scatter([],[],[], color='turquoise', alpha = 1,s=20)
        self.cloud = self.ax.scatter([],[],[], color='red', alpha = 1,s=20)
        self.stylus = self.ax.scatter([],[],[], color='red', alpha = 0.3)

        self.text1 = self.fig.text(0, 0.00, "NUMBER OF POINTS", va='bottom', ha='left',color='lightgrey',fontsize=12)  
        self.text2 = self.fig.text(0.5,0.97, "XYZ DATA", va='top', ha='center',color='lightgrey',fontsize=32)
        self.text3 = self.fig.text(0, 0.03, "NUMBER OF SAVED POINTS", va='bottom', ha='left', color='lightgrey', fontsize = 12)
        self.text4 = self.fig.text(0.5,0.85, "TIP ORIENTATION", va='bottom', ha='center', color='lightgrey', fontsize = 32)
        
        self.showPath = True
        self.showcsv = True
        self.showstylus = True

        self.ax.set_xlim3d(-500,500)
        self.ax.set_ylim3d(-500,500)
        self.ax.set_zlim3d(-50, 300)
        set_axes_equal(self.ax)

    def draw_cone(self, ax, x,y,z, dirx,diry,dirz ):
        # draw a cone with tip at x,y,z and orientation given by dirx,y,z
        height = 200
        theta = np.linspace(0, 2*np.pi, 10)
        r = np.linspace(0, 20, 10)
        t,R =np.meshgrid(theta, r)

        X = np.array(R*np.cos(t))
        Y = np.array(R*np.sin(t))
        Z = np.array(R*height/r.max())

        ihat = [1,0,0]
        jhat = [0,1,0]
        khat = [0,0,1]

        x_rot_M = self.rotation_matrix(ihat, dirx)
        y_rot_M = self.rotation_matrix(jhat, diry)
        z_rot_M = self.rotation_matrix(khat, dirz)

        Xprime = X
        Yprime = Y
        Zprime = Z

        # slice by slice rotation
        for i in range(len(X[:][1])):
            XYZprime = np.stack( [X[i][:], Y[i][:], Z[i][:]] , axis = 0)
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
        
        X = Xprime
        Y = Yprime
        Z = Zprime

        graph = ax.plot_surface(X+x, Y+y, Z+z,alpha=0.2, cmap=cm.GnBu)
        return graph
        pass

    def rotation_matrix(self, axis, theta):
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


    def init_plot(self):
        #print(">Init plot")

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
    
    def togglestylus(self):
        self.showstylus = not self.showstylus

    def update_lines(self, i=1, path=True, csv=True):
        # see original: https://stackoverflow.com/questions/50342300/animating-3d-scatter-plot-using-python-mplotlib-via-serial-data
        # I'm not using the iterator argument because my data is streaming from the serial port

        # use this block for clearing and redrawing. Not used with _offsets3d()
        self.init_plot()

        packet_received = False
        while(not packet_received):
            self.arm.ser.write(('>').encode('utf-8'))
            try:
                serial_rx = self.arm.ser.readline()
                try:
                    data = str(serial_rx[0:len(serial_rx)-2].decode("utf-8"))
                    if(data.startswith("XYZ")):
                        packet_received = True
                        xyz = data.split(",")       # Data format for Arduino: X Y Z alpha beta gamma (coords, stylus angles optional)
                        #print(xyz)
                        dx = float(xyz[1])
                        dy = float(xyz[2])
                        dz = float(xyz[3])    
                        dirx = float(xyz[4]) 
                        diry = float(xyz[5])
                        dirz = float(xyz[6])
                
                        self.text1.set_text("{:d} points in buffer".format(len(self.data.x)))  # for debugging
                        self.text2.set_text("(X,Y,Z): ({:.2f}, {:.2f}, {:.2f})".format(dx, dy, dz))  # for debugging
                        self.text3.set_text("{:d} points in point cloud".format(len(self.data.savex)))
                        self.text4.set_text("(Roll, Pitch, Yaw): ({:.2f}, {:.2f}, {:.2f})".format(dirx, diry, dirz))  # for debugging
                        packet_received = True
                        self.data.x.append(dx)
                        self.data.y.append(dy)
                        self.data.z.append(dz)
                        self.data.tipx = dx
                        self.data.tipy = dy
                        self.data.tipz = dz
                        self.data.dirx = dirx*np.pi/180
                        self.data.diry = diry*np.pi/180
                        self.data.dirz = dirz*np.pi/180

                        if len(self.data.x) > self.data.path_points:
                            self.data.x.pop(0)
                            self.data.y.pop(0)
                            self.data.z.pop(0)

                        #  You can update the plot with new data only
                        # graph._offsets3d = (x, y, z)
                        # graph2._offsets3d = (tipx,tipy,tipz)
                        
                        #  Or, if you can take the computation hit, redraw the whole plot. This way I get to draw lines too
                        self.tip = self.ax.scatter(dx,dy,dz,color='turquoise', alpha = 1,s=20)
                        if(self.showPath is True):
                            self.path = self.ax.plot(self.data.x,self.data.y,self.data.z, color='aquamarine', alpha = 0.3)
                        if(self.showcsv is True):
                            self.cloud = self.ax.scatter(self.data.savex, self.data.savey, self.data.savez, color='red', alpha=1, s=20)
                        if(self.showstylus is True):
                            self.stylus = self.draw_cone(self.ax, self.data.tipx,self.data.tipy,self.data.tipz, self.data.dirx,self.data.diry,self.data.dirz)

                        return self.path,self.tip,self.cloud,self.stylus
                    else:
                        pass
                except UnicodeDecodeError:
                    pass
            except serial.SerialException as e:
            #There is no new data from serial port
                pass

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
        #self.ani = animation.FuncAnimation(self.view.fig, self.view.update_lines, fargs = [], frames=100, interval=50, blit=False)     # this works. But it's not as flexible as controlling it myself

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
        root.bind("<space>", self.save_cloud_point)

    def  save_points(self):
        defaultname = 'point_cloud_'+time.strftime("%Y-%m-%d %H-%M-%S", time.localtime())+'.csv'
        f = asksaveasfile(initialfile = defaultname)            # read the docs. this returns an opened filewriter
        with f:
            for i in range(len(self.view.data.savex)):
                row = "%.2f, %.2f, %.2f\n" % (self.view.data.savex[i], self.view.data.savey[i], self.view.data.savez[i])
                print("Write row to file:" + row)
                f.write(row)

    def render(self):
        if(self.update):
            self.view.update_lines()                              # update fig,ax
            self.canvas.draw()                                    # send updated fig, ax to tkinder window. otherwise the update will not happen until I interact with the window
        self._renderjob = self.root.after(25, self.render)        # schedule updates with .after
    
    def toggle_render(self):
        self.update = not self.update

    def reset_plot(self):
        print(">Reset plot")
        self.view.data.clear_display_data()
        self.view.data.clear_csv_data()
        self.view.init_plot()
        self.view.update_lines()            # update plot
        self.canvas.draw()               # send updates to tkinder window
    
    def toggle_path(self):
        # self.view.data.toggle_path()
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
        self.view.data.clear_csv_data()
        self.view.init_plot()
        self.view.update_lines()            
        self.canvas.draw()              

    def save_cloud_point(self, e):
        #print(">Saved point to csv")
        self.view.data.savex.append(self.view.data.tipx)
        self.view.data.savey.append(self.view.data.tipy)
        self.view.data.savez.append(self.view.data.tipz)
        if(len(self.view.data.savex)>self.view.data.csv_points):
            self.view.data.savex.pop()
            self.view.data.savey.pop()
            self.view.data.savez.pop()
    
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
        self.arm = arm

if __name__ == "__main__":
    app = App()

    if(not app.view.arm.ser.is_open):
        print(">Opening serial port")
        app.view.arm.ser.open()
        app.view.arm.ser.flushInput()
        time.sleep(2)

    app.view.arm.wait_for_init()

    print(">Setting datastream mode")
    # app.view.arm.ser.write('c'.encode("utf-8"))      # turn on continuous reporting 
    app.view.arm.ser.write('q'.encode('utf-8'))       # turn on query-based reporting
    # app.view.arm.ser.write('>\n'.encode('utf-8'))
    app.view.arm.wait_for_response()

    app.gui.render()
    app.gui.root.mainloop()

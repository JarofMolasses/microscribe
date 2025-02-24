from matplotlib import cm
import mpl_toolkits
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import norm

import pytransform3d
from pytransform3d.transformations import transform_from, plot_transform
from pytransform3d.rotations import random_axis_angle, matrix_from_axis_angle
from pytransform3d.plot_utils import plot_cylinder, remove_frame

import math

linkcolor = 'slategrey'
jointcolor = 'lightgrey'


def plot_cylinder_wire(ax=None, length=1.0, radius=1.0, thickness=0.0,
                  A2B=None, start = np.array([0,0,0]), end = np.array([0,0,1]), x_s=1, wireframe=True, n_steps=100,
                  alpha=1.0, color="k"):
    """Plot cylinder.

    A cylinder is the volume covered by a disk moving along a line segment.

    Parameters
    ----------
    ax : Matplotlib 3d axis, optional (default: None)
        If the axis is None, a new 3d axis will be created

    length : float, optional (default: 1)
        Length of the cylinder

    radius : float, optional (default: 1)
        Radius of the cylinder

    thickness : float, optional (default: 0)
        Thickness of a cylindrical shell. It will be subtracted from the
        outer radius to obtain the inner radius. The difference must be
        greater than 0.

    A2B : array-like, shape (4, 4)
        Center of the cylinder

    ax_s : float, optional (default: 1)
        Scaling of the new matplotlib 3d axis

    wireframe : bool, optional (default: True)
        Plot wireframe of cylinder and surface otherwise

    n_steps : int, optional (default: 100)
        Number of discrete steps plotted in each dimension

    alpha : float, optional (default: 1)
        Alpha value of the cylinder that will be plotted

    color : str, optional (default: black)
        Color in which the cylinder should be plotted

    Returns
    -------
    ax : Matplotlib 3d axis
        New or old axis

    Raises
    ------
    ValueError
        If thickness is <= 0
    """
    inner_radius = radius - thickness
    if inner_radius <= 0.0:
        raise ValueError("Thickness of cylindrical shell results in "
                         "invalid inner radius: %g" % inner_radius)

    if(A2B is not None):
        axis_start = A2B.dot(np.array([0, 0, -0.5 * length, 1]))[:3]
        axis_end = A2B.dot(np.array([0, 0, 0.5 * length, 1]))[:3]
    else:
        axis_start = start
        axis_end = end

    axis = axis_end - axis_start
    length = norm(axis)
    axis /= length

    not_axis = np.array([1, 0, 0])
    if np.allclose(axis, not_axis) or np.allclose(-axis, not_axis):
        not_axis = np.array([0, 1, 0])

    n1 = np.cross(axis, not_axis)
    n1 /= norm(n1)
    n2 = np.cross(axis, n1)

    if wireframe:
        t = np.linspace(0, length, n_steps)
    else:
        t = np.array([0, length])
    theta = np.linspace(0, 2 * np.pi, n_steps)
    t, theta = np.meshgrid(t, theta)

    if thickness > 0.0:
        X_outer, Y_outer, Z_outer = [
            axis_start[i] + axis[i] * t
            + radius * np.sin(theta) * n1[i]
            + radius * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        X_inner, Y_inner, Z_inner = [
            axis_end[i] - axis[i] * t
            + inner_radius * np.sin(theta) * n1[i]
            + inner_radius * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        X = np.hstack((X_outer, X_inner))
        Y = np.hstack((Y_outer, Y_inner))
        Z = np.hstack((Z_outer, Z_inner))
    else:
        X, Y, Z = [axis_start[i] + axis[i] * t
                   + radius * np.sin(theta) * n1[i]
                   + radius * np.cos(theta) * n2[i] for i in [0, 1, 2]]

    # if wireframe:
    #     return ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10, alpha=alpha,
    #                       color=color)
    # else:
    #     return ax.plot_surface(X, Y, Z, color=color, alpha=alpha, linewidth=0)
    return X,Y,Z


# see: https://sabopy.com/py/matplotlib-3d-37/
def calculate_stylus_const(height=15, radius=3, height_total = 134):
    angle_res = 16
    height_res = 10
    theta = np.linspace(0, 2*np.pi, angle_res)
    r = np.linspace(0, radius, height_res)
    t,R =np.meshgrid(theta, r)
    X = np.array(R*np.cos(t))
    Y = np.array(R*np.sin(t))
    Z = np.array(R*height/radius)

    Xext = X[height_res-1, :]
    Yext = Y[height_res-1, :]
    Zext = np.ones(angle_res) * (height_total - height)

    X = np.vstack([X,Xext])
    Y = np.vstack([Y,Yext])
    Z = np.vstack([Z,Zext])
    return (X,Y,Z)

def recover_stylus_surface(XYZ,extension=True):
    angle_res = 16
    height_res = 10
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

def gen_stylus(Xc,Yc,Zc,endpoint, R):
    # get copy of cone points constants
    X = Xc.copy()
    Y = Yc.copy()
    Z = Zc.copy()

    x = endpoint[0]
    y = endpoint[1]
    z = endpoint[2]

    # execute rotation 
    XYZprime = np.stack( [X.ravel(), Y.ravel(), Z.ravel()] , axis = 0)
    XYZprime = np.dot(R, XYZprime)

    # plot surface
    (Xp,Yp,Zp) = recover_stylus_surface(XYZprime)             # get surface-able vectors from a plain XYZ point cloud 
    return Xp+x,Yp+y,Zp+z      

# this generates cylinder on two endpoints or at origin with a height.
def gen_cylinder_on_axis(radius = 20, height=10, p0_ = None, p1_ = None, radius_steps = 10, length_steps = 2):
    if(p0_ is None ):
        p0_ = np.array([0, 0, 0]) #point at one end
    p0 = p0_
    if(p1_ is None):
        p1_ = np.array([0, 0, height]) #point at other end
    p1 = p1_
    R = radius

    #vector in direction of axis
    v = p1 - p0

    #find magnitude of vector
    mag = norm(v)

    #unit vector in direction of axis
    v = v / mag

    #make some vector not in the same direction as v
    not_v = np.array([1, 0, 0])
    if (v == not_v).all():
        not_v = np.array([0, 1, 0])

    #make vector perpendicular to v
    n1 = np.cross(v, not_v)
    #normalize n1
    n1 /= norm(n1)

    #make unit vector perpendicular to v and n1
    n2 = np.cross(v, n1)

    #surface ranges over t from 0 to length of axis and 0 to 2*pi
    t = np.linspace(0, mag, length_steps)
    theta = np.linspace(0, 2 * np.pi, radius_steps)
    rsample = np.linspace(0, R, 2)

    #use meshgrid to make 2d arrays
    t, theta2 = np.meshgrid(t, theta)

    rsample,theta = np.meshgrid(rsample, theta)

    #generate coordinates for surface
    # "Tube"
    X, Y, Z = [p0[i] + v[i] * t + R * np.sin(theta2) * n1[i] + R * np.cos(theta2) *       n2[i] for i in [0, 1, 2]]
    # "Bottom"
    X2, Y2, Z2 = [p0[i] + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    # "Top"
    X3, Y3, Z3 = [p0[i] + v[i]*mag + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    # ax.plot_surface(X, Y, Z, color='blue', alpha = 0.9)
    # ax.plot_surface(X2, Y2, Z2, color='blue', alpha = 0.9)
    # ax.plot_surface(X3, Y3, Z3, color='blue', alpha = 0.9)
    return X,Y,Z,X2,Y2,Z2,X3,Y3,Z3

def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. Euclidean norm, along axis.

    >>> v0 = numpy.random.random(3)
    >>> v1 = unit_vector(v0)
    >>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
    True
    >>> v0 = numpy.random.rand(5, 4, 3)
    >>> v1 = unit_vector(v0, axis=-1)
    >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0 * v0, axis=2)), 2)
    >>> numpy.allclose(v1, v2)
    True
    >>> v1 = unit_vector(v0, axis=1)
    >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0 * v0, axis=1)), 1)
    >>> numpy.allclose(v1, v2)
    True
    >>> v1 = numpy.empty((5, 4, 3))
    >>> unit_vector(v0, axis=1, out=v1)
    >>> numpy.allclose(v1, v2)
    True
    >>> list(unit_vector([]))
    []
    >>> list(unit_vector([1]))
    [...1.0...]

    """
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.asarray(data)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data
    return None

def rotation_matrix(direction, angle, point=None):
    """Return matrix to rotate about axis defined by point and direction.

    >>> R = rotation_matrix(math.pi / 2, [0, 0, 1], [1, 0, 0])
    >>> numpy.allclose(numpy.dot(R, [0, 0, 0, 1]), [1, -1, 0, 1])
    True
    >>> angle = (random.random() - 0.5) * (2 * math.pi)
    >>> direc = numpy.random.random(3) - 0.5
    >>> point = numpy.random.random(3) - 0.5
    >>> R0 = rotation_matrix(angle, direc, point)
    >>> R1 = rotation_matrix(angle - 2 * math.pi, direc, point)
    >>> is_same_transform(R0, R1)
    True
    >>> R0 = rotation_matrix(angle, direc, point)
    >>> R1 = rotation_matrix(-angle, -direc, point)
    >>> is_same_transform(R0, R1)
    True
    >>> I = numpy.identity(4, numpy.float64)
    >>> numpy.allclose(I, rotation_matrix(math.pi * 2, direc))
    True
    >>> numpy.allclose(
    ...     2, numpy.trace(rotation_matrix(math.pi / 2, direc, point))
    ... )
    True

    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.diag([cosa, cosa, cosa])
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array(
        [
            [0.0, -direction[2], direction[1]],
            [direction[2], 0.0, -direction[0]],
            [-direction[1], direction[0], 0.0],
        ]
    )
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.asarray(point[:3], dtype=np.float64)
        M[:3, 3] = point - np.dot(R, point)
    return R


class LinkRender:
    def __init__(self, links = [], ax=None, fig=None):
        self.fig = fig
        self.ax = ax
        self.links = links
        self.plots_per_link = 8          # default number of plotting objects per link. 3 for the frame, 1 for the joint axis, 1 for the link 
        self.link_plots = [self.plots_per_link*[None] for i in range(len(self.links))]  
        self.stylus_plot = None
        # print("Initialized plotting objects: ")
        # print(self.link_plots)

        self.coneX, self.coneY, self.coneZ = calculate_stylus_const()

        for i in range(len(self.links)):
            self.link_plots[i][5] = ax.plot([],[],[],color = 'red')[0]
            self.link_plots[i][6] = ax.plot([],[],[],color = 'green')[0]
            self.link_plots[i][7] = ax.plot([],[],[],color = 'blue')[0]
        pass

    # don't run this if you have your axes predefined in another application.
    def init_render(self):
        ax = self.ax
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

    def clear_links(self):
        for i in range(len(self.links)):
                self.undraw_link(i)

    def undraw_link(self, link_index):                       # It works, but not certain about the memory freeing
        # print("Clearing link %d" % index)
        for i in range(len(self.link_plots[link_index])):
            plot_object = self.link_plots[link_index][i]

            # print("Deleting link %i index %i" % (link_index, i))
            # print(plot_object)
            if(plot_object is not None):
                # Try to delete surface
                if(type(plot_object) is mpl_toolkits.mplot3d.art3d.Poly3DCollection or type(plot_object) is mpl_toolkits.mplot3d.art3d.Line3DCollection):
                    try:
                        plot_object.remove()          # this works for 3D Surface (poly3Dcollection)
                        # print("Success")
                    except Exception as e:
                        print("Couldn't remove 3D collection")
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
                                print(repr(e))
                                print("Couldn't remove line3D")
                                pass
                    except:
                        pass

    def redraw_links(self):
        self.clear_links()
        self.draw_links()

    def gen_base_plane(self):
        xmin = -60
        xmax = 60
        ymin = -40
        ymax = 40
        [A,B,C,D] = [0,0,1,0]
        X = np.array([xmin, xmin, xmax, xmax])
        Y = np.array([ymin, ymax, ymin, ymax])
        Z = -(A*X + B*Y - D)/C
        return X,Y,Z
    
    def draw_stylus(self):
    # If it's the last link, draw special end effector graphics
        ax = self.ax
        link = self.links[-1]
        R = link.uvw
        endpoint = link.endpoint
        X,Y,Z = gen_stylus(self.coneX, self.coneY, self.coneZ, endpoint = endpoint, R=R)
        self.stylus_plot = ax.plot_surface(X,Y,Z, color=linkcolor, alpha = 0.8)

    def draw_links(self):
        ax = self.ax
        #self.link_plots = [self.plots_per_link*[None] for i in range(len(self.links))]  
        self.link_plots = [[] for i in range(8)]

        for i in range(len(self.links)):
            link = self.links[i]

            if(link.draw_link == True):
                # print("Drawing link %d" % i)
                if(link.child is not None):
                    link_p0 = link.endpoint
                    link_p1 = link.child.endpoint

                    # X = [link_p0[0], link_p1[0]]
                    # Y = [link_p0[1], link_p1[1]]
                    # Z = [link_p0[2], link_p1[2]]
                    # self.link_plots[i].append(ax.plot(X,Y,Z, color = 'red', alpha = 0.3))

                    #X,Y,Z,X2,Y2,Z2,X3,Y3,Z3 = gen_cylinder_on_axis(p0_ = link_p0, p1_=link_p1, radius = 5, radius_steps=8, length_steps = 4)
                    # self.link_plots[i].append(ax.plot_surface(X, Y, Z, color=linkcolor, alpha = 0.15))

                    X,Y,Z = plot_cylinder_wire(ax=ax, radius = 5, start = link_p0, end = link_p1, color = linkcolor, n_steps = 50, alpha = 0.15)
                    self.link_plots[i].append(ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10, alpha=0.3,color= linkcolor))

                    # self.link_plots[i][0] = ax.plot_surface(X, Y, Z, color=linkcolor, alpha = 0.15)

            if(link.draw_joint == True):
                # print("Drawing joint %d" % i)
                if(link.index != 0):                                    # Don't show a joint on the first frame, it's fixed.
                    joint_p0 = link.endpoint + 25*link.uvw[:,2]         # joint cylinder along z axis
                    joint_p1 = link.endpoint - 25*link.uvw[:,2]         

                    A2B = link.refframe

                    #X,Y,Z,X2,Y2,Z2,X3,Y3,Z3 = gen_cylinder_on_axis(p0_ = joint_p0, p1_=joint_p1, radius = 15, radius_steps = 8, length_steps = 4)
                    # self.link_plots[i].append(ax.plot_surface(X2, Y2, Z2, color=jointcolor, alpha = 0.6))
                    # self.link_plots[i].append(ax.plot_surface(X3, Y3, Z3, color=jointcolor, alpha = 0.6))
                    #self.link_plots[i].append(ax.plot_surface(X, Y, Z, color=jointcolor, alpha = 0.6))

                    X,Y,Z = plot_cylinder_wire(ax=ax, length = 50, radius = 15, A2B = A2B, color = jointcolor, n_steps = 50, alpha = 0.3)
                    self.link_plots[i].append(ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10, alpha=0.3,color= jointcolor))     # wireframe version. Faster?

                    # self.link_plots[i][1] = ax.plot_surface(X2, Y2, Z2, color=jointcolor, alpha = 0.6)
                    # self.link_plots[i][2] = ax.plot_surface(X3, Y3, Z3, color=jointcolor, alpha = 0.6)
                    # self.link_plots[i][3] = ax.plot_surface(X, Y, Z, color=jointcolor, alpha = 0.6)
           
                else:
                    X,Y,Z = self.gen_base_plane()
                    self.link_plots[i].append(ax.plot_trisurf(X,Y,Z,color = jointcolor, alpha = 0.5,edgecolor='none',linewidth=0))

                    #self.link_plots[i][4] = ax.plot_trisurf(X,Y,Z,color = jointcolor, alpha = 0.5,edgecolor='none',linewidth=0)
                    pass
            
            if(link.draw_frame == True):
                # print("Drawing frame %d" % (i))
                u = np.vstack([link.endpoint, 60*link.uvw[:,0] + link.endpoint])
                v = np.vstack([link.endpoint, 60*link.uvw[:,1] + link.endpoint])
                w = np.vstack([link.endpoint, 60*link.uvw[:,2] + link.endpoint])

                self.link_plots[i].append(ax.plot(u[:,0], u[:,1], u[:,2],color = 'red'))
                self.link_plots[i].append(ax.plot(v[:,0], v[:,1], v[:,2],color = 'green'))
                self.link_plots[i].append(ax.plot(w[:,0], w[:,1], w[:,2],color = 'blue'))

                # self.link_plots[i][5] = (ax.plot(u[:,0], u[:,1], u[:,2],color = 'red'))
                # self.link_plots[i][6] = (ax.plot(v[:,0], v[:,1], v[:,2],color = 'green'))
                # self.link_plots[i][7] = (ax.plot(w[:,0], w[:,1], w[:,2],color = 'blue'))

                # lineu = self.link_plots[i][5]
                # linev = self.link_plots[i][6]
                # linew = self.link_plots[i][7]
                # lineu.set_data_3d(u[:,0], u[:,1], u[:,2])
                # linev.set_data_3d(v[:,0], v[:,1], v[:,2])
                # linew.set_data_3d(w[:,0], w[:,1], w[:,2])

            self.ax.set_aspect('equal')

class Robot: 
    def __init__(self, links):
        self.links = links

    def set_angles(self, angles, base_offset = False):
        for i in range(len(angles)):
            try:
                if(base_offset == False):
                    self.links[i].set_angle(angles[i])
                else:
                    self.links[i+1].set_angle(angles[i])        # if the base frame is included in the model, then we need to offset the angles by one.
            except:
                pass

    def get_end_effector_endpoint(self) -> np.ndarray:
        return self.links[-1].endpoint

# class for keeping track of each joint's orientation and location. 
# each instance is responsible for a joint and the previous link.
# (LINK - JOINT) -> (LINK - JOINT)
class Link:
    def __init__(self, parent = None, alpha = 0, beta=0, commonnormal = 0, offsetpsi = 0, A = 0, D = 0, angle = 0, fixed = False, standard = False, draw_frame = False, draw_joint = True, draw_link = True):
        self.child = None
        origin_frame = np.eye(4,4)
        self.draw_joint = draw_joint
        self.draw_link = draw_link
        self.draw_frame = draw_frame
        self.fixed = fixed

        if(fixed == True): 
            # This means it's the origin link
            index = 0
        else:
            # This means we have to inherit the previous link's stuff
            origin_frame = parent.refframe
            index = parent.index + 1
            parent.child = self

        self.index = index
        self.A = A
        self.D = D
        self.alpha = alpha
        self.beta = beta
        self.offsetpsi = offsetpsi

        self.commonnormal = commonnormal
        self.parent = parent
        self.angle = angle

        T = np.eye(4,4)
        # Standard D-H
        if(standard == True):
            T_zdt = np.array(
                    [[np.cos(angle + offsetpsi), -np.sin(angle + offsetpsi), 0 , 0],
                    [np.sin(angle + offsetpsi), np.cos(angle + offsetpsi), 0, 0],
                    [0, 0, 1, D],
                    [0, 0, 0, 1]])
            
            T_xaa = np.array(
                    [[1, 0, 0, A],
                    [0, np.cos(alpha), -np.sin(alpha), 0],
                    [0, np.sin(alpha), np.cos(alpha), 0],
                    [0, 0, 0, 1]])
            T = np.dot(T_zdt, T_xaa)
        # Mod D-H
        else:
            c = np.cos(angle+offsetpsi)
            ca = np.cos(alpha)
            s = np.sin(angle+offsetpsi)
            sa = np.sin(alpha)
            if(self.index == 3):
                sb = np.sin(self.beta)
                cb = np.cos(self.beta)
                T = np.array(  [[c * cb,    -s * cb,   sb,     self.A + sb * self.D],
                            [s*ca + sa * sb * c, c*ca - sa * sb * s, -sa * cb, -sa * cb * self.D],
                            [s*sa - ca * sb * c, c*sa + s * sb * ca, ca * cb, cb * ca*self.D],
                            [0, 0, 0, 1]])
            else:
                T = np.array(   [[c, -s, 0, A],
                            [s*ca, c*ca, -sa, -sa * D],
                            [s*sa, c*sa, ca, ca*D],
                            [0, 0, 0, 1]])

            
        self.refframe = np.dot(origin_frame, T)
        # print("Frame %d: " %index)
        # print(self.refframe)
        self.uvw = self.refframe[:3, :3]
        self.endpoint = self.refframe[:3, 3]

    def calc_T_mod(self):
        # print("Recalculating frame %d" % self.index)
        c = np.cos(self.angle+self.offsetpsi)
        ca = np.cos(self.alpha)
        s = np.sin(self.angle+self.offsetpsi)
        sa = np.sin(self.alpha)
        if(self.index == 3):
            sb = np.sin(self.beta)
            cb = np.cos(self.beta)
            T = np.array(  [[c * cb,    -s * cb,   sb,     self.A + sb * self.D],
                            [s*ca + sa * sb * c, c*ca - sa * sb * s, -sa * cb, -sa * cb * self.D],
                            [s*sa - ca * sb * c, c*sa + s * sb * ca, ca * cb, cb * ca*self.D],
                            [0, 0, 0, 1]])
        else:
            T = np.array(   [[c, -s, 0, self.A],
                            [s*ca, c*ca, -sa, -sa * self.D],
                            [s*sa, c*sa, ca, ca*self.D],
                            [0, 0, 0, 1]])

        self.refframe = np.dot(self.parent.refframe, T)
        self.uvw = self.refframe[:3, :3]
        self.endpoint = self.refframe[:3, 3]
        if(self.child is not None):
            self.child.calc_T_mod()
        
    def set_angle(self, angle):
        self.angle = angle
        self.calc_T_mod()

if __name__ == "__main__":
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(6,6),facecolor = 'black')
    ax = fig.add_subplot(111, projection='3d')
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    link0 = Link(fixed = True)                                # origin frame
    link1 = Link(parent = link0, D = 210.82)
    link2 = Link(parent = link1, D=-22.25, A =24.31, alpha = np.pi/2)
    link3 = Link(parent = link2, D=-0.03, A = 260.40)
    link4 = Link(parent = link3, D=234.85, A=13.89, alpha = np.pi/2)
    link5 = Link(parent = link4, D=8.13, A = -10.16, alpha = -np.pi/2)
    link6 = Link(parent = link5, D=-134.01,A = 10.16, alpha = -np.pi/2, draw_joint = False)
    links = [link0,link1,link2,link3,link4,link5,link6]
    robot = Robot(links)
    renderer = LinkRender(links, ax = ax, fig = fig)

    renderer.init_render()
    renderer.draw_links()
    renderer.draw_stylus()
    robot.set_angles([0.00000,2.39033,5.26539,0.00000,4.89800], base_offset = True)
    renderer.redraw_links() 
    renderer.draw_stylus()

    # robot.set_angles([0.00000,0,0,0,0], base_offset = True)
    # renderer.redraw_links() 
    

    
    
    plt.show()
    pass
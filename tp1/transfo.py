import numpy as np
import pinocchio as pin


def t3d(x,y,z,qx,qy,qz,qw):
    '''Convert a 7d vector (translation-quaternion) into a 4x4 homogeneous matrix as expected by MeshCat. '''
    T = np.eye(4)
    T[0,3] = x
    T[1,3] = y
    T[2,3] = z
    q = pin.Quaternion(qw,qx,qy,qz)
    q.normalize()
    T[:3,:3] = q.matrix()
    return T

def t2d(x, y, theta):
    '''Convert a 3d vector (x,y,theta) into a transformation in the Y,Z plane.'''
    s,c=np.sin(theta/2),np.cos(theta / 2)
    return t3d(0, x, y, s,0,0,c)  # Rotation around X

def translation(x,y,z):
    '''Convert a 3d vector (x,y,z) into a pure translation. '''
    T = np.eye(4)
    T[0,3] = x
    T[1,3] = y
    T[2,3] = z
    return T

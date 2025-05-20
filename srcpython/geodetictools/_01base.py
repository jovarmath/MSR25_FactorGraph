import numpy as np
from scipy.interpolate import interp1d

# Table of contents #

# (1) ##### Vector Simplifications ##### #
# (1.1) vector2(x,y) ... vector 2D simplification
# (1.1) vector3(x,y,z) ... vector 3D simplification
# (1.2) vector4(x,y,z) ... vector 4D simplification (e.g. Quaternion)

# (2) ##### Unit Transformations ##### #
# (2.1) radiant to degree conversion
# (2.2) degree to radiant conversion
#
# (3)  ##### 2D Rotations ##### #
# TODO: if needed

# (4)  ##### 3D Rotations ##### #
# (4.1) Rotation Matrix X from angle
# (4.2) Rotation Matrix Y from angle
# (4.3) Rotation Matrix Z from angle
# (4.4) Matrix XY Reflexion 
# (4.5) Matrix YZ Reflexion 
# (4.6) Matrix XZ Reflexion
# (4.7) Rotmat2Euler --> 
# (4.8) vec2skewmat( x ): --- vector to skew symmetric martix
# (4.9) quat_mult( p, q ): --- quaterion multiplication 
# (4.10) quat_change( sigK ): --- change in rotation as quaternion
# (4.11) Quat2Euler( q ): --- Quaternion to Euler angles 
# (4.12) Euler2Quat(r,p,y): --- Euler angles to Quaternion
# (4.13) R = quat2Rotmat( q ) Quaternion to Rotation matrix conversion
# (4.14) q = Rotmat2quat( R ) Rotation Matrix to Quaternion conversion

# =================================================================================
# (1) Vector Simplifications
def vector2(x,y):
    return np.array((x,y), dtype=float)
# (1.1) 3D float vector
def vector3(x,y,z):
    return np.array((x,y,z), dtype=float)
# (1.2) 4D float vector
def vector4(w,x,y,z):
    return np.array((w,x,y,z), dtype=float)
# (1.3) matrix 3x3 float
def matrix33(x00,x01,x03,x10,x11,x12,x20,x21,x22):
    return np.array([[x00,x01,x03],[x10,x11,x12],[x20,x21,x22]], dtype=float)

# Find duplicates indices in vector
def find_duplicate( vector ):
    _, unique_indices, counts = np.unique( vector, return_index=True, return_counts=True)
    duplicate_indices = unique_indices[counts > 1]
    return duplicate_indices


# =================================================================================
# (2) Unit Transformations
#
# (2.1) radiant to degree conversion
def rad2deg(angle_rad):
# Input: angle in radiant
# Output: angle in degree
    return ( angle_rad * (180/np.pi) )

# (2.2) degree to radiant conversion
def deg2rad(angle_deg):
# Input: angle in degree
# Output: angle in radiant
    return ( angle_deg * (np.pi/180) )

# =================================================================================
# (3) 2D Rotations
# TODO
#
#
# =================================================================================
# (4) 3D Rotations
#
#
#
# 4.1 Rotation Matrix X from Angle Xr
def RotmatX( alpha ):
    return np.array([ [1,0,0] , [0, np.cos(alpha), -np.sin(alpha)], [0, np.sin(alpha), np.cos(alpha)] ])

# 4.2 Rotation Matrix Y from Angle Yr
def RotmatY( beta ):
    return np.array([ [np.cos(beta), 0, np.sin(beta) ], [0,1,0], [-np.sin(beta), 0, np.cos(beta)] ])

# 4.3 Rotation Matrix Z from Angle Zr
def RotmatZ( gamma ):
    return np.array([ [np.cos(gamma),-np.sin(gamma), 0] , [np.sin(gamma), np.cos(gamma), 0] , [0,0,1] ])

# 4.4  Matrix XY Reflexion 
def ReflectXYplane():
    return np.array([ [1, 0, 0] , [0, 1, 0] , [0, 0, -1] ])

# 4.5  Matrix YZ Reflexion
def ReflectYZplane():
    return np.array([ [-1, 0, 0] , [0, 1, 0] , [0, 0, 1] ])

# 4.6  Matrix XZ Reflexion 
def ReflectXZplane():
    return np.array([ [1, 0, 0] , [0, -1, 0] , [0, 0, 1] ])


# (4.7) Rotmat2Euler( rotmat ) --- Rotation Matrix to Euler angles
def Rotmat2Euler( rotmat ):
# ############################################################################
# Function computes the Euler Angles from given rotation matrix
# ----------------------------------------------------------------------------
# Input:
# rotmat (double 3x3)...matrix
#-----------------------------------------------------------------------------
# @author: Felix Esser
# @date: 14.07.2020
# @mail: s7feesse@uni-bonn.de
# @ literature: Förstner and Wrobel (2016), Photogrammetric Computer Vision
# ############################################################################

    rX  = np.arctan2( rotmat[2,1], rotmat[2,2] )
    rY = np.arctan2( -rotmat[2,0], np.sqrt(rotmat[2,1]**2 + rotmat[2,2]**2) )
    rZ   = np.arctan2( rotmat[1,0], rotmat[0,0] )

    rXrYrZ = vector3( rX, rY, rZ )

    return rXrYrZ

# (4.8) vec2skewmat( x ): --- vector to skew symmetric martix
def vec2skewmat( x ):
# ############################################################################
# Function computes the skew matrix from a given vector
# ----------------------------------------------------------------------------
# Input:
# v (double 3x1)...vector
# Output:
# S_x (double 3x3)...matrix
#-----------------------------------------------------------------------------
# @author: Felix Esser
# @date: 15.07.2020
# @mail: s7feesse@uni-bonn.de
# @literature: Förstner & Wrobel (2016), p.201
# ############################################################################

    S_x = np.zeros((3,3))

    S_x[0,0] =    0
    S_x[0,1] = -x[2]
    S_x[0,2] =  x[1]

    S_x[1,0] =  x[2]
    S_x[1,1] =    0
    S_x[1,2] = -x[0]

    S_x[2,0] = -x[1]
    S_x[2,1] =  x[0]
    S_x[2,2] =    0

    return S_x

# (4.9) quat_mult( p, q ): --- quaterion multiplication 
def quat_mult( q, r ):
# ############################################################################
# function computes the product of two quaternions, which applys the change in Orientation
# see Groves 2013 Appendix_E.pdf p.10
# author: Felix Esser
# date: 14.07.20
# multiplication: p = q * r, update of orientation with quaternions
#
# literatur: Förstner & Wrobel 2016, Photogrammetric Computer Vision, p.334, formula  (8.43)
# ############################################################################
    
    p = np.zeros((4))
    p[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3]
    p[1] = q[1]*r[0] + q[0]*r[1] - q[3]*r[2] + q[2]*r[3]
    p[2] = q[2]*r[0] + q[3]*r[1] + q[0]*r[2] - q[1]*r[3]
    p[3] = q[3]*r[0] - q[2]*r[1] + q[1]*r[2] + q[0]*r[3]

    return p


# (4.10) quat_change( sigK ): --- change in rotation as quaternion
def quat_change( sigK ):
# ############################################################################
#  computes the Quaternion that represents the change in rotation
#  
#  date: 14.07.20
#  author: Felix Esser
#  E-mail: s7feesse@uni-bonn.de
#  literature: Grooves 2013, Appendix
# ############################################################################
    n = np.linalg.norm( sigK )

    rK = np.zeros((4))

    if (n > 10**-10):
        rK[0] =  np.cos(n/2)
        rK[1:4]  = (sigK/n) * (np.sin(n/2))
    else:
        rK[0] = 1 - (1/(2*(2**2)))*(n**2) + (1/(24*(2**4)))*(n**4) - (1/(720*(2**6)))*(n**6)
        rK[1:4] = sigK * (0.5 - (1/(6*(2**3)))*(n**2) + (1/(120*(2**5)))*(n**4) - (1/(5040*(2**7)))*(n**6) )
    return rK

# vc

# n=norm(dsig); % norm
# if n>1e-10
#    r=[cos(n/2) ; dsig ./n.* sin(n/2)]; %(3.116)
# else
#    den=@(x) factorial(x)*2^x;   
#    r=[1-1/den(2)*n.^2 + 1/den(4)*n.^4 - 1/den(6)*n.^6;
#        dsig.*repmat(0.5 - 1/den(3)*n.^2 + 1/den(5)*n.^4 - 1/den(7)*n.^6,3,1)]'; %(3.117)
# end

# (4.11) Quat2Euler( q ): --- Quaternion to Euler angles 
def Quat2Euler( q ):
# ############################################################################
#  computes roll pitch yaw angle from quaternion 
# 
#  date: 13.07.20
#  author: Felix Esser
#  E-mail: s7feesse@uni-bonn.de
# ############################################################################

    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    rX = np.arctan2(2*(q0*q1+q2*q3),(1-2*q1**2-2*q2**2))
    rY = np.arcsin(2*(q0*q2-q1*q3))
    rZ = np.arctan2(2*(q0*q3+q1*q2),(1-2*q2**2-2*q3**2))

    return rX, rY, rZ

# (4.12) Euler2Quat(r,p,y): --- Euler angles to Quaternion
def Euler2Quat(rX,rY,rZ):
# ############################################################################
# 
# Function computes the Quaternion from Euler angles 
#  
# INPUT:    r,p,y:    orientation: roll, pitch und yaw ( EULER Angles ) [RAD]
# 
# OUTPUT:   q [4x1]:  orientation: quaternion
# 
# author: Felix Esser
# date:   13.07.20
# 
# ############################################################################

    # % Aufstellen des Quaternion
    q = np.zeros( ( 4, 1 ) )

    q[0,0] = np.cos(rX/2)*np.cos(rY/2)*np.cos(rZ/2) + np.sin(rX/2)*np.sin(rY/2)*np.sin(rZ/2)
    q[1,0] = np.sin(rX/2)*np.cos(rY/2)*np.cos(rZ/2) - np.cos(rX/2)*np.sin(rY/2)*np.sin(rZ/2)
    q[2,0] = np.cos(rX/2)*np.sin(rY/2)*np.cos(rZ/2) + np.sin(rX/2)*np.cos(rY/2)*np.sin(rZ/2)
    q[3,0] = np.cos(rX/2)*np.cos(rY/2)*np.sin(rZ/2) - np.sin(rX/2)*np.sin(rY/2)*np.cos(rZ/2)



    # old matlab code:
    # q = [ np.cos(r/2)*np.cos(p/2)*np.cos(y/2) + np.sin(r/2)*np.sin(p/2)*np.sin(y/2)
    #       np.sin(r/2)*np.cos(p/2)*np.cos(y/2) - np.cos(r/2)*np.sin(p/2)*np.sin(y/2)
    #       np.cos(r/2)*np.sin(p/2)*np.cos(y/2) + np.sin(r/2)*np.cos(p/2)*np.sin(y/2)
    #       np.cos(r/2)*np.cos(p/2)*np.sin(y/2) - np.sin(r/2)*np.sin(p/2)*np.cos(y/2) ];

    return q


def changeQuat( dsig ):

    n=np.linalg.norm(dsig) 

    t123 = (dsig/n) * np.sin(n/2)

    r= vector4(np.cos(n/2), t123[0], t123[1], t123[2] ) #(3.116)

    return r


# (4.13) R = quat2Rotmat( q ) Quaternion to Rotation matrix conversion
def quat2Rotmat( q ):
# ############################################################################
# Determine the rotation matrix R that corresponds to Quaternion
# ----------------------------------------------------------------------------
# Input:
# q 
# Output:
# R
#-----------------------------------------------------------------------------
# @author: Felix Esser
# @date: 14.07.2020
# @mail: s7feesse@uni-bonn.de
# literature: Förstner & Wrobel (2016), Photogrammetric Computer Vision, p. 335, formula 8.55
# ############################################################################

    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]

    R = np.zeros((3,3))
          
    # DCM Matrix (body => navigation!!):    
    # DCM = n*[(a^2+b^2-c^2-d^2)   2*(b*c-a*d)         2*(b*d+a*c);...
    #          2*(b*c+a*d)        (a^2-b^2+c^2-d^2)   2*(c*d-a*b);...
    #           2*(b*d-a*c)         2*(c*d+a*b)       (a^2-b^2-c^2+d^2)];       


    R[0,0] = a**2+b**2-c**2-d**2
    R[0,1] = 2*(b*c-a*d)
    R[0,2] = 2*(b*d+a*c)

    R[1,0] = 2*(b*c+a*d)
    R[1,1] = (a**2-b**2+c**2-d**2)
    R[1,2] = 2*(c*d-a*b)

    R[2,0] = 2*(b*d-a*c) 
    R[2,1] = 2*(c*d+a*b)
    R[2,2] = (a**2-b**2-c**2+d**2)

    # R = 1/(np.linalg.norm(q)**2)*R --> this line is missing in matlab

    return R

    # matlab code 
    # DCM = [(a^2+b^2-c^2-d^2)   2*(b*c-a*d)         2*(b*d+a*c);...
    #           2*(b*c+a*d)        (a^2-b^2+c^2-d^2)   2*(c*d-a*b);...
    #           2*(b*d-a*c)         2*(c*d+a*b)       (a^2-b^2-c^2+d^2)];

# (4.14) q = Rotmat2quat( R ) Rotation Matrix to Quaternion conversion
def Rotmat2quat( R ):

    # to euler angles
    rXrYrZpy = Rotmat2Euler( R )

    # euler to quat 
    q = Euler2Quat( rXrYrZpy[0], rXrYrZpy[1], rXrYrZpy[2] )

    return q
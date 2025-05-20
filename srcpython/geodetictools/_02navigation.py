import numpy as np



# (1) Navigation
# (1.1) Rotation Matrix from roll pitch yaw 
# (1.2) Rotation Matrix Navigation Frame to Earth Frame from lattitude longitude
# (1.3) Rotation matrix R_be from Body to Earth Frame




# (1.1) Rotation Matrix from roll pitch yaw 
def get_R_bn( roll, pitch, yaw ):

    # body to navigataion frame 
    # east north up!

    sy = np.sin(yaw)
    sp = np.sin(pitch)
    sr = np.sin(roll)

    cy = np.cos(yaw)
    cp = np.cos(pitch)
    cr = np.cos(roll)

    R_b_n = np.array([[cp*cy , -cr*sy+sr*sp*cy , sr*sy+cr*sp*cy ], [cp*sy , cr*cy+sr*sp*sy , -sr*cy+cr*sp*sy], [-sp , sr*cp , cr*cp]], dtype = float)

    return R_b_n


# (1.2) Rotation Matrix Navigation Frame to Earth Frame from lattitude longitude
#
#
def get_R_ne( L, B, frame):
# ======================================================================= %
# Determine the Rotation matrix from the navigation- to the ecef-frame 
# ----------------------------------------------------------------------- %
# Input:
# lam (1x1 double) [rad] ... longitude, Längengrad L
# phi (1x1 double) [rad] ... latitude, Breitengrad B

# East Nort Up (see Jekeli 2001, page 25 but note, this is for a NED)
# NED: Groves 2013 p.74, (NED ... north east down)
#--------------------------------------------------------------------------
# @author: Felix Esser
# @date: 24.07.2020
# @mail: s7feesse@uni-bonn.de
# Literature: Jekeli - intertial navigation systems (2001)


# ======================================================================= %
	if (frame == 'NED'): # north east down
		C_ne = np.array( [[0,0,0], [0,0,0], [0,0,0]] , dtype=float)

		C_ne[0,0] = -np.sin(B)*np.cos(L)
		C_ne[0,1] = -np.sin(B)
		C_ne[0,2] = -np.cos(B)*np.cos(L)

		C_ne[1,0] = -np.sin(B)*np.sin(L)
		C_ne[1,1] = np.cos(L)
		C_ne[1,2] = -np.cos(B)*np.sin(L)

		C_ne[2,0] = np.cos(B)
		C_ne[2,1] = 0
		C_ne[2,2] = -np.sin(B)	


	elif (frame == 'ENU'): # north east up frame

		# sin-terms
		sl = np.sin(L)
		sb = np.sin(B)

		# cos-terms
		cl = np.cos(L)
		cb = np.cos(B)

		# from matlab
		#       East   North    Up
		# C_ne = [-sl   -sb*cl    cb*cl;...
		#          cl   -sb*sl    cb*sl;...
		#          0       cb     sb  ];

		C_ne = np.array( [[1,1,1], [1,1,1], [1,1,1]] , dtype=float)

		C_ne[0,0] = -sl
		C_ne[0,1] = -sb*cl 
		C_ne[0,2] =  cb*cl

		C_ne[1,0] = cl
		C_ne[1,1] = -sb*sl
		C_ne[1,2] = cb*sl

		C_ne[2,0] = 0
		C_ne[2,1] = cb
		C_ne[2,2] = sb
         
	return C_ne 


# (1.3) Rotation matrix R_be from Body to Earth Frame

def R_be(roll, pitch, yaw, lon, lat):
# ======================================================================= %
# Determination of the Rotation matrix from the body - to the ecef-frame 
# ----------------------------------------------------------------------- %
# Input:
# roll (1x1 double) [rad]
# pitch (1x1 double) [rad]
# yaw (1x1 double) [rad]
# lam (1x1 double) [rad] ... longitude
# phi (1x1 double) [rad] ... latitude
# note: code modified to python code from matlab (Felix Esser)

#--------------------------------------------------------------------------
# @author: Christian Eling
# @date: 27.05.2014
# @mail: eling@igg.uni-bonn.de
# ======================================================================= %

    # Rotation matrix body => navigation:
	C_bn = get_R_bn( roll, pitch, yaw )
	#q_bn = Rotmat2quat( C_bn )

	# Rotation matrix navigation ENU => ecef:
	C_ne = get_R_ne(lon,lat, 'NEU')
	#q_ne = Rotmat2quat( C_ne )

	# Rotation matrix body => ecef: B --> E
	C_be = C_ne @ C_bn
	#q_be = quat_mult( q_ne, q_bn )

	# quaternion test
	#R_test = quat2Rotmat( q_be )

	# Euler angles for the e-frame:
	roll_e =  np.arctan2( C_be[2,1], C_be[2,2] )
	pitch_e = np.arcsin( -C_be[2,0] )
	yaw_e =   np.arctan2( C_be[1,0], C_be[0,0] )


	return C_be, roll_e, pitch_e, yaw_e

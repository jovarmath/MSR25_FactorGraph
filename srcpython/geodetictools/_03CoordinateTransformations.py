import numpy as np
import os, sys

# current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add parent folder to path
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from srcpython.geodetictools import _01base as geobase


# (1) ##### Coordinate Transformations #####
# (1.1) ECEF-Frame to LLA Transformation
# (1.2) Reference Ellipsoid Parameter function
# (1.3) Ellipsoidical Coordinates to UTM Transformation
# (1.4) UTM to ellipsoidal coordiantes
# (1.5) UTM to local topocentric ellsipsoid system

# (1) ##### Frame Transformations ##### #

# (1.1) ECEF-Frame to LLA Transformation

def ecef2lla(x, y, z, ellipsoid, version = 1):

    # ecef2lla - convert earth-centered earth-fixed (ECEF)
    #            cartesian coordinates to latitude, longitude,
    #            and altitude (above WGS84 ellipsoid)

    # INPUT 
    # x = ECEF X-coordinate (m)
    # y = ECEF Y-coordinate (m)
    # z = ECEF Z-coordinate (m)

    # OUTPUT
    # lat = geodetic latitude (radians) B
    # lon = longitude (radians) L
    # alt = height above WGS84 ellipsoid (m)

    # modified: 08.07.20 by Felix Esser
    # - vector transformation now possible

    # a = 6378137
    e = 8.1819190842622e-2

    a, b, e2, finv = refell( ellipsoid )

    #% calculations
    #b = np.sqrt(a**2*(1-e**2))

    if version == 1:

        ep = np.sqrt((a**2-b**2)/b**2)

        p = np.sqrt(np.power(x, 2) + np.power(y, 2))
        th = np.arctan2(a*z, b*p)
        lon = np.arctan2(y, x)

        # lattitude --> Breitengrad B
        lat = np.arctan2((z+np.power(ep,2)*b*np.power(np.sin(th),3)), (p-np.power(e,2)*a*np.power(np.cos(th),3)))


        N = a / np.sqrt( 1 - np.power(e,2) * np.power(np.sin(lat),2) )

        alt = p/np.cos(lat)-N

        # longitude --> Längengrad L
        lon = np.fmod(lon, 2*np.pi)# East North Upd(lon, 2*np.pi)

        #% correct for numerical instability in altitude near exact poles:
        #% (after this correction, error is about 2 millimeters, which is about
        #% the same as the numerical precision of the overall function)

        if ( (type(x) == type(np.array([1])))): # array of coordinates
            for i in range(0,len(x)):
                if (np.fabs(x[i]) < 1 and np.fabs(y[i]) < 1):
                    alt[i] = np.fabs(z[i])-b
        elif ( (type(x) == type(np.matrix([1]))) ):
            for i in range(0,len(x)):
                if (np.fabs(x[i]) < 1 and np.fabs(y[i]) < 1):
                    alt[i] = np.fabs(z[i])-b

        else: # single coordinate
            if (np.fabs(x) < 1 and np.fabs(y) < 1):
                    alt = np.fabs(z)-b

    #return lat, lon, alt


    # --------------------------------------------------------------
    # VERSION 2 
    # --------------------------------------------------------------

    if version == 2:

        elat=1.e-12
        eht=1.e-5


        p = np.sqrt( x**2 + y**2 )
        lat=np.arctan2( z, p / (1-e2) )
        h=0
        dh=1
        dlat=1

        counter = 1

        while (np.sum( dlat ) > elat ) | (np.sum( dh ) > eht ): 
            lat0 = lat
            h0 = h
            v = a / np.sqrt( 1 - e2 * np.sin( lat ) * np.sin( lat ) )
            h = p / np.cos( lat ) - v
            lat = np.arctan2( z, p * ( 1 - e2 * v / (v + h)))
            dlat = np.absolute( lat - lat0 )
            dh = np.absolute( h - h0 )

            counter += 1
        
        lon = np.arctan2(y,x)
        
        alt = h

    return (lat, lon, alt)


# (1.2) Reference Ellipsoid function
def refell(sys):
# modified by: Felix Esser, 20.07.20
# REFELL  Computes reference ellispoid parameters.
#   TOPEX Reference: <http://topex-www.jpl.nasa.gov/aviso/text/general/news/hdbk311.htm#CH3.3>
# Version: 1 Jun 04
# Useage:  [a,b,e2,finv]=refell(type)
# Input:   type - reference ellipsoid type (char)
#                 CLK66 = Clarke 1866
#                 GRS67 = Geodetic Reference System 1967
#                 GRS80 = Geodetic Reference System 1980
#                 WGS72 = World Geodetic System 1972
#                 WGS84 = World Geodetic System 1984
#                 ATS77 = Quasi-earth centred ellipsoid for ATS77
#                 NAD27 = North American Datum 1927 (=CLK66)
#                 NAD83 = North American Datum 1927 (=GRS80)
#                 INTER = International
#                 KRASS = Krassovsky (USSR)
#                 MAIRY = Modified Airy (Ireland 1965/1975)
#                 TOPEX = TOPEX/POSEIDON ellipsoid
# Output:  a    - major semi-axis (m)
#          b    - minor semi-axis (m)
#          e2   - eccentricity squared
#          finv - inverse of flattening

    if (sys =='CLK66' or sys =='NAD27'):
      a=6378206.4
      finv=294.9786982
    elif sys =='GRS67':
      a=6378160.0
      finv=298.247167427
    elif (sys =='GRS80' or sys =='NAD83'):
      a=6378137.0
      finv=298.257222101
    elif (sys =='WGS72'):
      a=6378135.0
      finv=298.26
    elif (sys =='WGS84'):
      a=6378137.0
      finv=298.257223563
    elif sys =='ATS77':
      a=6378135.0
      finv=298.257
    elif sys =='KRASS':
      a=6378245.0
      finv=298.3
    elif sys =='INTER':
      a=6378388.0
      finv=297.0
    elif sys =='MAIRY':
      a=6377340.189
      finv=299.3249646
    elif sys =='TOPEX':
      a=6378136.3
      finv=298.257

    f=1/finv
    b=a*(1-f)
    e2=1-(1-f)**2

    return a, b, e2, finv

# _____________________________________________________________________________________
# (1.3) Ellipsoidical Coordinates to UTM Transformation
#
#

def ell2utm( lat, lon, Zone: int = 32):
# function [N,E,Zone]=ell2utm(lat,lon,a,e2,lcm)
# ELL2UTM  Converts ellipsoidal coordinates to UTM.
#   UTM northing and easting coordinates in a 6 degree
#   system.  Zones begin with zone 1 at longitude 180E
#   to 186E and increase eastward.  Formulae from E.J.
#   Krakiwsky, "Conformal Map Projections in Geodesy",

# Useage:  [N,E,Zone]=ell2utm(lat,lon,a,e2,lcm)
#          [N,E,Zone]=ell2utm(lat,lon,a,e2)
#          [N,E,Zone]=ell2utm(lat,lon,lcm)
#          [N,E,Zone]=ell2utm(lat,lon)
# Input:   lat - vector of latitudes (rad)
#          lon - vector of longitudes (rad)
#          a   - ref. ellipsoid major semi-axis (m); default GRS80
#          e2  - ref. ellipsoid eccentricity squared; default GRS80
#          lcm - central meridian; default = standard UTM def'n
# Output:  N   - vector of UTM northings (m)
#          E   - vector of UTM eastings (m)
#          Zone- vector of UTM zones

    # parameter reference ellipsoid
    [a, b, e2, finv] = refell('WGS84')

    lcm = (Zone*6-183) * (np.pi/180)    # ML --> lcm=deg2rad(Zone*6-183);


    ko=0.9996                 # Scale factor
    No=np.zeros( lat.shape )  # False northing (north)
    # No(lat<0)=1e7           # False northing (south)
    np.where(lat < 0, No, 1e7)
    Eo=500000                 # False easting

    lam=lon-lcm



    np.where(lam >= np.pi, lam, lam - 2*np.pi) # lam=lam-(lam >= pi)*(2*pi)
      
    #fprintf('\nZones\n');
    #fprintf('%3d\n',Zone');
    #fprintf('\nCentral Meridians\n');
    #fprintf('%3d %2d %9.6f\n',rad2dms(lcm)');
    #fprintf('\nLongitudes wrt Central Meridian\n');
    #fprintf('%3d %2d %9.6f\n',rad2dms(lam)');

    f = 1 - np.sqrt(1 - e2)

    RN = a / ((1 - e2 * np.sin(lat)**2))**0.5
    RM = a*(1-e2)/(1-e2 * np.sin(lat)**2)**1.5
    t = np.tan(lat)
    h = np.sqrt((e2*np.cos(lat)**2) / (1 - e2))
    n = f/(2-f)

    a0=1+n**2 / 4+n**4 / 64
    a2=1.5*(n-n**3/8)
    a4=15/16*(n**2-n**4/4)
    a6=35/48*n**3
    a8=315/512*n**4

    s=a/(1+n)*(a0*lat-a2*np.sin(2*lat)+a4*np.sin(4*lat)-a6*np.sin(6*lat)+a8*np.sin(8*lat))

    E1=lam * np.cos(lat)
    E2=lam**3 * np.cos(lat)**3/6 * (1-t**2+h**2)
    E3=lam**5 * np.cos(lat)**5/120 * (5-18*t**2+t**4+14*h**2-58*t**2*h**2+13*h**4+4*h**6-64*t**2*h**4-24*t**2*h**6)
    E4=lam**7 * np.cos(lat)**7/5040*(61-479*t**2+179*t**4-t**6)

    E=Eo + ko*RN*(E1 + E2 + E3 + E4)

    N1=lam**2/2 * np.sin(lat) * np.cos(lat)
    N2=lam**4/24 * np.sin(lat) * np.cos(lat)**3*(5-t**2+9*h**2+4*h**4)
    N3=lam**6/720 * np.sin(lat) * np.cos(lat)**5*(61-58*t**2+t**4+270*h**2-330*t**2*h**2+445*h**4+324*h**6-680*t**2*h**4+88*h**8-600*t**2*h**6-192*t**2*h**8)
    N4=lam**8/40320*np.sin(lat)*np.cos(lat)**7*(1385-311*t**2+543*t**4-t**6)

    N=No + ko*RN*(s/RN + N1 + N2 + N3 + N4)

    return N,E


# _____________________________________________________________________________________
# (1.4) UTM to ellipsoidal coordinates
#
#

def utm2ell(N,E,Zone):
    # get parameters of WGS84 Ellipsoid
    [a, b, e2, finv] = refell('WGS84')
    f = 1 / finv

    #----- Remove false northings and eastings
    No=np.zeros(N.shape)   # False northing (north)
    #No(Zone<0)=1e7      # False northing (south)
    Eo=500000           # False easting
    N=N-No
    E=E-Eo
    Zone=np.absolute(Zone)   # Remove negative zone indicator for southern hemisphere

    #----- Foot point latitude
    ko=0.9996           # UTM scale factor
    lat1=N/ko/a
    dlat=1

    lcm=geobase.deg2rad(np.absolute(Zone)*6-183)

    while np.amax(np.absolute(dlat))>1e-12:
        A0=1-(e2/4)-(e2**2*3/64)-(e2**3*5/256)-(e2**4*175/16384)
        A2=(3/8)*( e2+(e2**2/4)+(e2**3*15/128)-(e2**4*455/4096) )
        A4=(15/256)*( e2**2+(e2**3*3/4)-(e2**4*77/128) )
        A6=(35/3072)*( e2**3-(e2**4*41/32) )
        A8=-(315/131072)*e2**4
        f1=a*( A0*lat1-A2*np.sin(2*lat1)+A4*np.sin(4*lat1)-A6*np.sin(6*lat1)+A8*np.sin(8*lat1) )-N/ko
        f2=a*( A0-2*A2*np.cos(2*lat1)+4*A4*np.cos(4*lat1)-6*A6*np.cos(6*lat1)+8*A8*np.cos(8*lat1) )

        dlat=-f1/f2
        lat1=lat1+dlat

    RN=a/(1-e2*np.sin(lat1)**2)**0.5

    RM=a*(1-e2)/(1-e2*np.sin(lat1)**2)**1.5
    h2=e2*np.cos(lat1)**2/(1-e2)
    t=np.tan(lat1)

    E0=E/ko/RN
    E1=E0
    E2=E0**3/6*(1+2*t**2+h2)
    E3=E0**5/120*(5+6*h2+28*t**2-3*h2**2+8*t**2*h2+24*t**4-4*h2**3+4*t**2.*h2**2+24*t**2*h2**3)
    E4=E0**7/5040*(61 + 662*t**2 + 1320*t**4 + 720*t**6)
    lon=(1/np.cos(lat1))*(E1-E2+E3-E4)+lcm

    E0=E/ko
    N1=(t*E0**2)/(2*RM*RN)
    N2=(t*E0**4)/(24*RM*RN**3)*(5+3*t**2+h2-4*h2**2-9*h2*t**2)
    N3=(t*E0**6)/(720*RM*RN**5)*(61-90*t**2+46*h2+45*t**4-252*t**2*h2-5*h2**2+100*h2**3-66*t**2*h2**2-90*t**4.*h2+88*h2**4+225*t**4.*h2**2+84*t**2*h2**3 - 192*t**2*h2**4)
    N4=(t*E0**8)/(40320*RM*RN**7)*(1385+3633*t**2+4095*t**4+1575*t**6)
    lat=lat1-N1+N2-N3+N4

    return lat, lon

# _____________________________________________________________________________________
# (1.5) Ellipsoidal coordinates to global cartesian
#
#

def ell2xyz(lat,lon,h):
  [a, b, e2, finv] = refell('WGS84')

  v=a/np.sqrt(1-e2*np.sin(lat)*np.sin(lat))
  x=(v+h)*np.cos(lat)*np.cos(lon)
  y=(v+h)*np.cos(lat)*np.sin(lon)
  z=(v*(1-e2)+h)*np.sin(lat)

  return x,y,z

# _____________________________________________________________________________________
# (1.6) UTM to local topocentric (ellipsoid)
#
#

def utm2topocentric(x,y,z, T=None): 

  # Ellipsoidal coordinates
  [lat, lon] = utm2ell(y,x,32)
 
  # Global cartesian
  x,y,z = ell2xyz(lat,lon,z)

  # Rotation
  B = np.mean(lat[0:100])
  L = np.mean(lon[0:100])

  if T is None or (isinstance(T, np.ndarray) and T.size == 0):
    Rot = np.array([ [-np.sin(B) * np.cos(L), -np.sin(B) * np.sin(L), np.cos(B)],
                    [-np.sin(L), np.cos(L), 0],
                    [np.cos(B) * np.cos(L), np.cos(B) * np.sin(L), np.sin(B)] ])

    # Translation
    xyz_m = np.array((np.mean(x[0:100]), np.mean(y[0:100]), np.mean(z[0:100]) ))

    # Homogeneous transformation matrix
    T = np.zeros((4, 4))
    T[:3, :3] = Rot
    T[:3, 3] = xyz_m
    T[3, 3] = 1
  else:
    T   = np.asarray(T) 
    Rot = T[:3, :3]
    xyz_m = T[:3, 3]
  
  xyz_loc = (Rot @ (np.c_[x, y, z] - xyz_m).T).T

  x = np.array(xyz_loc[:,1])  
  y = np.array(xyz_loc[:,0])
  z = np.array(xyz_loc[:,2])

  return np.c_[x,y,z], T

# _____________________________________________________________________________________
# (1.6) local topocentric (ellipsoid) to UTM
#
#

def topocentric2utm( xyz, T ):

  # Rotationsmatrix und Mittelwert laden

  xyz[:, [0, 1]] = xyz[:, [1, 0]]

  #Rot_loaded = np.loadtxt(route_gnss + 'rotationsmatrix.txt')
  #mean_loaded = np.loadtxt(route_gnss + 'meanpoint.txt')

  Rot = T[:3, :3]
  t = T[:3, 3]

  # Transformationen zurückberechnen

  xyz_rotated_back = xyz @ Rot
  xyz_transformed = xyz_rotated_back + t  # Verschiebung rückgängig machen

  # Leere Listen für lat, lon, alt erstellen
  lat_list, lon_list, alt_list = [], [], []

  for i in range(xyz_transformed.shape[0]):

    lat_val, lon_val, alt_val = ecef2lla( xyz_transformed[i, 0], xyz_transformed[i, 1], xyz_transformed[i, 2], "WGS84")

    lat_list.append(lat_val)
    lon_list.append(lon_val)
    alt_list.append(alt_val)

  # Listen in Arrays umwandeln

  lat = np.array(lat_list)
  lon = np.array(lon_list)
  alt = np.array(alt_list)

  # UTM-Transformation durchführen
  E, N = ell2utm(lat, lon, 32)

  # UTM-Koordinaten als XYZ-Array für Rücktransformation erstellen

  xyz_utm = np.vstack((N, E, alt)).T
  xyz = xyz_utm

  #self.trajectory.statesall[:,1:4] = xyz

  return xyz

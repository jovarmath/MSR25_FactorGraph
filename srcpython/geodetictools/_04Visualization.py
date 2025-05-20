import numpy as np
import matplotlib
import pandas as pd
import os
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QVBoxLayout, QWidget

# Import geobase functions
from srcpython.geodetictools import _01base as geobase

# (1) Visualization Functions 
# (1.1) createKML( lat, lon, h, type )

def createKML( lat, lon, alt, dimension , typeflag, color, filename ):

# imput:
# lat ... latitude in [rad]
# lon ... longitude in [rad]
# h 

# ----------------------------------------------------------------------            
# 2D
# ----------------------------------------------------------------------

    if (dimension == "2D"):
        if (typeflag == "linestring"):
            # Write kml data to file
            text_file = open(filename, "w")
            text_file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            text_file.write('<kml xmlns="http://earth.google.com/kml/2.0">\n')
            text_file.write('<Document>\n')
            text_file.write('   <Placemark>\n')
            text_file.write('      <LineString>\n')
            text_file.write('         <coordinates>\n')
            for i in range(0,len(lat)):
                text_file.write('            '+str(geobase.rad2deg( lon[i] ))+', '+str(geobase.rad2deg( lat[i] ))+', 0.0 '+'\n')
            text_file.write('         </coordinates>\n')
            text_file.write('      </LineString>\n')
            text_file.write('         <Style>\n')
            text_file.write('            <LineStyle>\n')
            text_file.write('               <color>#ff0000ff</color>\n')
            text_file.write('               <width>5</width>\n')
            text_file.write('            </LineStyle>\n')
            text_file.write('         </Style>\n')      
            text_file.write('   </Placemark>\n')
            text_file.write('</Document>\n')
            text_file.write('</kml>\n')
            text_file.close()

        elif (typeflag == "point"):

            # color
            if (color == "green"):
                CL = 'https://upload.wikimedia.org/wikipedia/commons/e/eb/Gruener_Punkt.png'
            elif (color == "beige"):
                CL = "https://upload.wikimedia.org/wikipedia/commons/8/89/Beiger_Punkt.png"

            # Write kml data to file
            text_file = open(filename, "w")
            text_file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            text_file.write('<kml xmlns="http://earth.google.com/kml/2.0">\n')
            text_file.write('<Document>\n')

            # define styles 
            text_file.write('      <Style id="custom">\n')
            text_file.write('         <IconStyle>\n')
            text_file.write('            <scale>0.3</scale>\n')
            text_file.write('               <Icon>\n')
            text_file.write('                   <href> '+ CL +' </href>\n')
            text_file.write('               </Icon>\n')
            text_file.write('         </IconStyle>\n')
            text_file.write('      </Style>\n')

            text_file.write('      <Style id="custom2">\n')
            text_file.write('         <LabelStyle>\n')
            text_file.write('            <color>ffff0000</color>\n')
            text_file.write('            <colorMode>normal</colorMode>\n')
            text_file.write('            <scale>1</scale>\n')
            text_file.write('         </LabelStyle>\n')
            text_file.write('      </Style>\n')

            '''
            <Style id="randomColorIcon">
                <IconStyle>
                    <color>8a2be2</color>
                    <colorMode>random</colorMode>
                    <scale>1.1</scale>
                    <Icon>
                        <href>http://maps.google.com/mapfiles/kml/pal3/icon21.png</href>
                    </Icon>
                </IconStyle>
            </Style>
            '''

            for i in range( 0, len(lat) ):
                text_file.write('   <Placemark>\n')
                text_file.write('      <styleUrl>custom</styleUrl>\n')
                text_file.write('      <Point> \n')
                text_file.write('         <coordinates> \n')
                text_file.write('            '+str(geobase.rad2deg( lon[i] ))+', '+str(geobase.rad2deg( lat[i]))+', 0.0 '+'\n')
                text_file.write('         </coordinates> \n')
                text_file.write('      </Point> \n')
                text_file.write('   </Placemark>\n')

            text_file.write('</Document>\n')
            text_file.write('</kml>\n')
            text_file.close()


# ----------------------------------------------------------------------            
# 3D
# ----------------------------------------------------------------------
    if (dimension == "3D"):

        if (typeflag == "linestring"):
            # Write kml data to file
            text_file = open(filename, "w")
            text_file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            text_file.write('<kml xmlns="http://earth.google.com/kml/2.0">\n')
            text_file.write('<Document>\n')
            text_file.write('   <Placemark>\n')
            text_file.write('      <LineString>\n')
            text_file.write('         <coordinates>\n')
            for i in range(0,len(lat)):
                text_file.write('            '+str(geobase.rad2deg( lon[i] ))+', '+str(geobase.rad2deg( lat[i] ))+', '+ str( alt[i] ) +'\n')
                text_file.write('         </coordinates>\n')
                text_file.write('      </LineString>\n')
                text_file.write('         <Style>\n')
                text_file.write('            <LineStyle>\n')
                text_file.write('               <color>#ff0000ff</color>\n')
                text_file.write('               <width>5</width>\n')
                text_file.write('            </LineStyle>\n')
                text_file.write('         </Style>\n')      
                text_file.write('   </Placemark>\n')
                text_file.write('</Document>\n')
                text_file.write('</kml>\n')
                text_file.close()

        elif (typeflag == "point"):
            # color
            if (color == "green"):
                CL = 'https://upload.wikimedia.org/wikipedia/commons/e/eb/Gruener_Punkt.png'
            elif (color == "beige"):
                CL = "https://upload.wikimedia.org/wikipedia/commons/8/89/Beiger_Punkt.png"

            # Write kml data to file
            text_file = open(filename, "w")
            text_file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            text_file.write('<kml xmlns="http://earth.google.com/kml/2.0">\n')
            text_file.write('<Document>\n')

            # define styles 
            text_file.write('      <Style id="custom">\n')
            text_file.write('         <IconStyle>\n')
            text_file.write('            <scale>0.3</scale>\n')
            text_file.write('               <Icon>\n')
            text_file.write('                   <href> '+ CL +' </href>\n')
            text_file.write('               </Icon>\n')
            text_file.write('         </IconStyle>\n')
            text_file.write('      </Style>\n')

            text_file.write('      <Style id="custom2">\n')
            text_file.write('         <LabelStyle>\n')
            text_file.write('            <color>ffff0000</color>\n')
            text_file.write('            <colorMode>normal</colorMode>\n')
            text_file.write('            <scale>1</scale>\n')
            text_file.write('         </LabelStyle>\n')
            text_file.write('      </Style>\n')

            '''
            <Style id="randomColorIcon">
                <IconStyle>
                    <color>8a2be2</color>
                    <colorMode>random</colorMode>
                    <scale>1.1</scale>
                    <Icon>
                        <href>http://maps.google.com/mapfiles/kml/pal3/icon21.png</href>
                    </Icon>
                </IconStyle>
            </Style>
            '''

            for i in range( 0, len(lat) ):
                text_file.write('   <Placemark>\n')
                text_file.write('      <styleUrl>custom</styleUrl>\n')
                text_file.write('      <Point> \n')
                text_file.write('         <coordinates> \n')
                text_file.write('            '+str(geobase.rad2deg( lon[i] ))+', '+str(geobase.rad2deg( lat[i]))+', '+ str( alt[i] ) + '\n')
                text_file.write('         </coordinates> \n')
                text_file.write('      </Point> \n')
                text_file.write('   </Placemark>\n')

            text_file.write('</Document>\n')
            text_file.write('</kml>\n')
            text_file.close()
                


# (2) save trajectory data 
# (2.1) for trajectopy

def write_trajectory_for_trajectopy(Data, out_dir: str, name: str = 'current'):
    print("Write trajectory in file for trajectopy (.traj)")
    os.makedirs(out_dir, exist_ok=True)
    out_filename = os.path.join(out_dir, f"GTSAM_{name}.traj")
    with open(out_filename, "w", newline="\n") as file:
        file.write(f"#name GTSAM_{name}\n")
        file.write(f"#epsg 25832\n")
        file.write(f"#fields t,px,py,pz,ex,ey,ez,vx,vy,vz\n")
        dataframme = pd.DataFrame(
            np.c_[
                Data[:,0],
                Data[:,1],
                Data[:,2],
                Data[:,3],
                Data[:,7],
                Data[:,8],
                Data[:,9],
                Data[:,4],
                Data[:,5],
                Data[:,6]
            ]
        ) 
    dataframme.to_csv(out_filename, index=False ,sep=",", header=False,mode="a")



# (7) Plot Functions
# # (7.1) Plot 2D Pose 
# # (7.2) Plot 3D Pose



# (7.2) Plot 3D Pose

def plotPose3D( xyz, rpy, axis_length, frame, ax):
    
    # Reference Frame of RPY is important:
    # can be given in ENU and NED frame !!!

    if frame == "ENU":

        R_BN = geobase.RotmatZ(rpy[2]) @ geobase.RotmatY(rpy[1]) @ geobase.RotmatX(rpy[0])

        x_axis = xyz + R_BN[0,:] * axis_length
        y_axis = xyz + R_BN[1,:] * axis_length
        z_axis = xyz + R_BN[2,:] * axis_length

        x_X = [ xyz[0], x_axis[0] ]
        x_Y = [ xyz[1], x_axis[1] ]
        x_Z = [ xyz[2], x_axis[2] ]

        y_X = [ xyz[0], y_axis[0] ]
        y_Y = [ xyz[1], y_axis[1] ]
        y_Z = [ xyz[2], y_axis[2] ]

        z_X = [ xyz[0], z_axis[0] ]
        z_Y = [ xyz[1], z_axis[1] ]
        z_Z = [ xyz[2], z_axis[2] ]


    elif frame == "NED":

        R_NED_ENU = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
        R_BN = R_NED_ENU @ geobase.RotmatZ(rpy[2]) @ geobase.RotmatY(rpy[1]) @ geobase.RotmatX(rpy[0])

        x_axis = xyz + R_BN[0,:] * axis_length
        y_axis = xyz + R_BN[1,:] * axis_length
        z_axis = xyz + R_BN[2,:] * axis_length

        x_X = [ xyz[0], x_axis[0] ]
        x_Y = [ xyz[1], x_axis[1] ]
        x_Z = [ xyz[2], x_axis[2] ]

        y_X = [ xyz[0], y_axis[0] ]
        y_Y = [ xyz[1], y_axis[1] ]
        y_Z = [ xyz[2], y_axis[2] ]

        z_X = [ xyz[0], z_axis[0] ]
        z_Y = [ xyz[1], z_axis[1] ]
        z_Z = [ xyz[2], z_axis[2] ]
    

    ax.plot3D( x_X, x_Y, x_Z, "-r" )
    ax.plot3D( y_X, y_Y, y_Z, "-g" )
    ax.plot3D( z_X, z_Y, z_Z, "-b" )



# (7.2) Plot 3D Pose

def plotPose2D( xyz, rpy, axis_length, frame):
    
    # Reference Frame of RPY is important:
    # can be given in ENU and NED frame !!!

    if frame == "ENU":

        R_BN = geobase.RotmatZ(rpy[2]) @ geobase.RotmatY(rpy[1]) @ geobase.RotmatX(rpy[0])

        x_axis = xyz + R_BN[0,:] * axis_length
        y_axis = xyz + R_BN[1,:] * axis_length
        z_axis = xyz + R_BN[2,:] * axis_length

        x_X = [ xyz[0], x_axis[0] ]
        x_Y = [ xyz[1], x_axis[1] ]
        x_Z = [ xyz[2], x_axis[2] ]

        y_X = [ xyz[0], y_axis[0] ]
        y_Y = [ xyz[1], y_axis[1] ]
        y_Z = [ xyz[2], y_axis[2] ]

        z_X = [ xyz[0], z_axis[0] ]
        z_Y = [ xyz[1], z_axis[1] ]
        z_Z = [ xyz[2], z_axis[2] ]


    elif frame == "NED":

        R_NED_ENU = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
        R_BN = R_NED_ENU @ geobase.RotmatZ(rpy[2]) @ geobase.RotmatY(rpy[1]) @ geobase.RotmatX(rpy[0])

        x_axis = xyz + R_BN[0,:] * axis_length
        y_axis = xyz + R_BN[1,:] * axis_length
        z_axis = xyz + R_BN[2,:] * axis_length

        x_X = [ xyz[0], x_axis[0] ]
        x_Y = [ xyz[1], x_axis[1] ]
        x_Z = [ xyz[2], x_axis[2] ]

        y_X = [ xyz[0], y_axis[0] ]
        y_Y = [ xyz[1], y_axis[1] ]
        y_Z = [ xyz[2], y_axis[2] ]

        z_X = [ xyz[0], z_axis[0] ]
        z_Y = [ xyz[1], z_axis[1] ]
        z_Z = [ xyz[2], z_axis[2] ]
    

    plt.plot(x_X, x_Y, "-r")
    plt.plot(y_X, y_Y, "-g")



class PlotTabs:
    def __init__(self, window_title: str = "PlotTabs"):
        #if fig_size == np.array(0,0):
        
        fig_size = [1280,900]

        plt.figure()
        self.app = QApplication.instance()
        self.MainWindow = QMainWindow()
        self.tabs = QTabWidget()

        self.MainWindow.__init__()
        self.MainWindow.setWindowTitle(window_title)
        self.MainWindow.setCentralWidget(self.tabs)
        self.MainWindow.resize(fig_size[0], fig_size[1])
        self.MainWindow.show()

        self.canvases = []
        self.figure_handles = []
        self.toolbar_handles = []
        self.tab_handles = []
        self.current_window = -1

    def addPlot(self, title, figure):
        new_tab = QWidget()
        layout = QVBoxLayout()
        new_tab.setLayout(layout)

        new_canvas = FigureCanvas(figure)
        new_toolbar = NavigationToolbar(new_canvas, new_tab)

        layout.addWidget(new_canvas)
        layout.addWidget(new_toolbar)
        self.tabs.addTab(new_tab, title)

        self.toolbar_handles.append(new_toolbar)
        self.canvases.append(new_canvas)
        self.figure_handles.append(figure)
        self.tab_handles.append(new_tab)

    def show(self):
        self.app.exec_()
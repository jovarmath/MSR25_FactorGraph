import json
import logging
import coloredlogs
from collections import namedtuple

class SigInitial:
    def __init__(self, sig_xyz, sig_vxyz, sig_rpy):
        self.sig_xyz = sig_xyz
        self.sig_vxyz = sig_vxyz
        self.sig_rpy = sig_rpy

class iSAM:
    def __init__(self, relinearizeSkip, relinearizeThreshold, poserate, updaterate):
        self.relinearizeSkip = relinearizeSkip
        self.relinearizeThreshold = relinearizeThreshold
        self.poserate = poserate
        self.updaterate = updaterate

# Create a logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG) 

coloredlogs.install(level='DEBUG', logger=logger)

class iSAMConfig:
    def __init__(self):
        
        # sigma acceleration
        self.useGPS: bool = True
        self.useGPSheading = True
        self.useTS = True
        self.useTSheading = True

        # sigma acceleration
        self.initial = SigInitial( sig_xyz = [0,0,0], sig_vxyz = [0,0,0], sig_rpy = [0,0,0] )

        self.initialtime: float = 0

        self.preintegrationSig = [0,0,0]

        self.isamconfig = iSAM( relinearizeSkip=0, relinearizeThreshold=0, poserate=0, updaterate=0 )

    # ##################################################################################
    # Setter functions

    # Acceleration
    def set_sig_xyz(self, sig_xyz ):
        self.initial( sig_xyz = sig_xyz )

    def set_preintegrationSig(self, preintsig):
        self.preintegrationSig = preintsig

    # ##################################################################################
    # Write function

    def writetojson(self, path, filename):

        # Construct the full path to the JSON file
        filepath = path + filename

        #print(f"Writing .json config file: {filepath}")

        # Create a dictionary structure to hold all the data
        data = {
            "MeasurementUpdates": {
                "useGPS": self.useGPS,
                "useGPSheading": self.useGPSheading,
                "useTS": self.useTS,
                "useTSheading": self.useTSheading,
            },
            "Initialsig": {
                "position": {
                    "x": self.initial.sig_xyz[0],
                    "y": self.initial.sig_xyz[1],
                    "z": self.initial.sig_xyz[2],
                },
                "velocity": {
                    "x": self.initial.sig_vxyz[0],
                    "y": self.initial.sig_vxyz[1],
                    "z": self.initial.sig_vxyz[2],
                },
                "orientation": {
                    "x": self.initial.sig_rpy[0],
                    "y": self.initial.sig_rpy[1],
                    "z": self.initial.sig_rpy[2],
                },
            },
            "InitialTime": self.initialtime,
            "PreintegrationSig": {
                "x": self.preintegrationSig[0],
                "y": self.preintegrationSig[1],
                "z": self.preintegrationSig[2],
            },
            "iSAM": {
                "relinearizeSkip": self.isamconfig.relinearizeSkip,
                "relinearizeThreshold": self.isamconfig.relinearizeThreshold,
                "poserate": self.isamconfig.poserate,
                "updaterate": self.isamconfig.updaterate,
            },
        }

        # Write the data to a JSON file
        with open(filepath, 'w') as json_file:
            json.dump(data, json_file, indent=4)




















    # ##################################################################################
    # Read function

    def readfromjson(self, path, filename):
        # Construct the full path to the JSON file
        filepath = path + filename

        print( "Reading .json IMU configfile: " + path + filename )
        
        # Open and read the JSON file
        with open(filepath, 'r') as json_file:
            data = json.load(json_file)

        # Access specific elements
        self.useGPS = data["MeasurementUpdates"]["useGPS"]
        self.useGPSheading = data["MeasurementUpdates"]["useGPSheading"]
        self.useTS = data["MeasurementUpdates"]["useTS"]
        self.useTSheading = data["MeasurementUpdates"]["useTSheading"]

        self.initial = SigInitial( sig_xyz = [data["Initialsig"]["position"]["x"],data["Initialsig"]["position"]["y"],data["Initialsig"]["position"]["z"]],
                                   sig_vxyz = [data["Initialsig"]["velocity"]["x"],data["Initialsig"]["velocity"]["y"],data["Initialsig"]["velocity"]["z"]],
                                   sig_rpy = [data["Initialsig"]["orientation"]["x"],data["Initialsig"]["orientation"]["y"],data["Initialsig"]["orientation"]["z"]] )
        
        self.initialtime = data["InitialTime"]

        self.preintegrationSig = [data["PreintegrationSig"]["x"],
                                  data["PreintegrationSig"]["y"],
                                  data["PreintegrationSig"]["z"]]

        self.isamconfig.relinearizeSkip = data["iSAM"]["relinearizeSkip"]
        self.isamconfig.relinearizeThreshold = data["iSAM"]["relinearizeThreshold"]
        self.isamconfig.poserate = data["iSAM"]["poserate"]
        self.isamconfig.updaterate = data["iSAM"]["updaterate"]

        P = (
            "\n ______________________________________________________"
            "\n| --------------- iSAM config ------------------------ |"
            "\n| Measurement updates:                                 |"
            f"\n| - useGPS:                                           {self.useGPS:.0f}|"
            f"\n| - useGPSheading:                                    {self.useGPSheading:.0f}|"
            f"\n| - useTS:                                            {self.useTS:.0f}|"
            f"\n| - useTSheading:                                     {self.useTSheading:.0f}|"
            "\n|                                                      |"
            "\n| Initial stochastics:                                 |"
            f"\n| - Position:       {self.initial.sig_xyz[0]:.2f}, {self.initial.sig_xyz[1]:.2f}, {self.initial.sig_xyz[2]:.2f}  [m]              |"
            f"\n| - Velocity:       {self.initial.sig_vxyz[0]:.2f}, {self.initial.sig_vxyz[1]:.2f}, {self.initial.sig_vxyz[2]:.2f}   [m/s]           |"
            f"\n| - Orientation:       {self.initial.sig_rpy[0]:.2f}, {self.initial.sig_rpy[1]:.2f}, {self.initial.sig_rpy[2]:.2f}   [rad]        |\n"
            f"\n| Initial time:                                   {self.initialtime:.2f}|\n"
            f"\n| - Preintegration sigma:      {self.preintegrationSig[0]:.2f}, {self.preintegrationSig[1]:.2f}, {self.preintegrationSig[2]:.2f}  [-]   |\n"
            "\n| iSAM config:                                         |"
            f"\n| - relinearizeSkip:                                  {self.isamconfig.relinearizeSkip:.0f}|"
            f"\n| - relinearizeThreshold:                        {self.isamconfig.relinearizeThreshold:.4f}|"
            f"\n| - poserate:                                    {self.isamconfig.poserate:.4f}|"
            f"\n| - updaterate:                                      {self.isamconfig.updaterate:.0f}|"
            "\n|______________________________________________________|"
        )

        print(P)
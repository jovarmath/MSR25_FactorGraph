import json
import logging
import coloredlogs

# Create a logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG) 

coloredlogs.install(level='DEBUG', logger=logger)

class IMUStochasticParameter:
    def __init__(self):
        
        # sigma acceleration
        self.sigaccx: float = 0
        self.sigaccy: float = 0
        self.sigaccz: float = 0

        # sigma acceleration
        self.siggyrox: float = 0
        self.siggyroy: float = 0
        self.siggyroz: float = 0

        # sigma bias acceleration
        self.sigbiasaccx: float = 0
        self.sigbiasaccy: float = 0
        self.sigbiasaccz: float = 0
        
        # sigma bias gyroscope
        self.sigbiasgyrox: float = 0
        self.sigbiasgyroy: float = 0
        self.sigbiasgyroz: float = 0

    # ##################################################################################
    # Setter functions

    # Acceleration
    def setsigaccxyz(self, sigacc ):
        self.sigaccx = sigacc[0]
        self.sigaccy = sigacc[1]
        self.sigaccz = sigacc[2]
    
    # Gyroscope
    def setsiggyroxyz(self, siggyro ):
        self.siggyrox = siggyro[0]
        self.siggyroy = siggyro[1]
        self.siggyroz = siggyro[2]

    # Acceleration bias
    def setsigbiasaccxyz(self, sigbiasacc ):
        self.sigbiasaccx = sigbiasacc[0]
        self.sigbiasaccy = sigbiasacc[1]
        self.sigbiasaccz = sigbiasacc[2]

    # Gyroscope bias
    def setsigbiasgyroxyz(self, sigbiasgyro ):
        self.sigbiasgyrox = sigbiasgyro[0]
        self.sigbiasgyroy = sigbiasgyro[1]
        self.sigbiasgyroz = sigbiasgyro[2]

    # ##################################################################################
    # Write function

    def writetojson(self, path, filename):
        
        filepath = path + filename
        #print(f"Writing .json config file: {filepath}")

        data = {
                "sigacc": {
                            "x": self.sigaccx,
                            "y": self.sigaccy,
                            "z": self.sigaccz,
                          },
                "siggyro": {
                            "x": self.siggyrox,
                            "y": self.siggyroy,
                            "z": self.siggyroz,
                           },
                "sigaccbias": {
                            "x": self.sigbiasaccx,
                            "y": self.sigbiasaccy,
                            "z": self.sigbiasaccz,
                           },
                "siggyrobias": {
                            "x": self.sigbiasgyrox,
                            "y": self.sigbiasgyroy,
                            "z": self.sigbiasgyroz,
                           }
                }

        # Write the data to a JSON file
        with open(path + filename, 'w') as json_file:
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
        self.sigaccx = data["sigacc"]["x"]
        self.sigaccy = data["sigacc"]["y"]
        self.sigaccz = data["sigacc"]["z"]

        self.siggyrox = data["siggyro"]["x"]
        self.siggyroy = data["siggyro"]["y"]
        self.siggyroz = data["siggyro"]["z"]

        self.sigbiasaccx = data["sigaccbias"]["x"]
        self.sigbiasaccy = data["sigaccbias"]["y"]
        self.sigbiasaccz = data["sigaccbias"]["z"]

        self.sigbiasgyrox = data["siggyrobias"]["x"]
        self.sigbiasgyroy = data["siggyrobias"]["y"]
        self.sigbiasgyroz = data["siggyrobias"]["z"]

        P = (
            f"\n ______________________________________________________\n"
            f"| --------------- IMU stochastic Info ---------------- |\n"
            f"| IMU acceleration sigma x:           "+"{:.6f}".format(self.sigaccx)         +" [m/s^2] |\n"
            f"| IMU acceleration sigma y:           "+"{:.6f}".format(self.sigaccy)         +" [m/s^2] |\n"
            f"| IMU acceleration sigma z:           "+"{:.6f}".format(self.sigaccz)         +" [m/s^2] |\n"
            f"|                                                      |\n"
            f"| IMU gyroscope sigma x:              "+"{:.6f}".format(self.siggyrox)         +" [rad/s] |\n"
            f"| IMU gyroscope sigma y:              "+"{:.6f}".format(self.siggyroy)         +" [rad/s] |\n"
            f"| IMU gyroscope sigma z:              "+"{:.6f}".format(self.siggyroz)         +" [rad/s] |\n"
            f"|                                                      |\n"
            f"| IMU acceleration bias sigma x:      "+"{:.6f}".format(self.sigbiasaccx)         +" [m/s^2] |\n"
            f"| IMU acceleration bias sigma y:      "+"{:.6f}".format(self.sigbiasaccy)         +" [m/s^2] |\n"
            f"| IMU acceleration bias sigma z:      "+"{:.6f}".format(self.sigbiasaccz)         +" [m/s^2] |\n"
            f"|                                                      |\n"
            f"| IMU gyroscope bias sigma x:         "+"{:.6f}".format(self.sigbiasgyrox)         +" [rad/s] |\n"
            f"| IMU gyroscope bias sigma y:         "+"{:.6f}".format(self.sigbiasgyroy)         +" [rad/s] |\n"
            f"| IMU gyroscope bias sigma z:         "+"{:.6f}".format(self.sigbiasgyroz)         +" [rad/s] |\n"
            f"|______________________________________________________|\n"
        )

        print(P)
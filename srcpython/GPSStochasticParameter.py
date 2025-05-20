import json
import logging
import coloredlogs

# Create a logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG) 

coloredlogs.install(level='DEBUG', logger=logger)

class GPSStochasticParameter:
    def __init__(self):
        
        # sigma position [m]
        self.sigposx: float = 0
        self.sigposy: float = 0
        self.sigposz: float = 0

    def setsigposxyz(self, sigposx, sigposy, sigposz):
        self.sigposx = sigposx
        self.sigposy = sigposy
        self.sigposz = sigposz

    # ##################################################################################
    # Write function

    def writetojson(self, path, filename):

        #p("Write IMU config file: ", path + filename)
        logger.info("Writing .json GPS configfile")

        data = {
                "sigpos": {
                            "x": self.sigposx,
                            "y": self.sigposy,
                            "z": self.sigposz,
                          }
                }

        # Write the data to a JSON file
        with open(path + filename, 'w') as json_file:
            json.dump(data, json_file, indent=4)

    def readfromjson(self, path, filename):
        # Construct the full path to the JSON file
        filepath = path + filename

        logger.info( "Reading .json GPS configfile: " + path + filename )
        
        # Open and read the JSON file
        with open(filepath, 'r') as json_file:
            data = json.load(json_file)

        # Access specific elements
        self.sigposx = data["sigpos"]["x"]
        self.sigposy = data["sigpos"]["y"]
        self.sigposz = data["sigpos"]["z"]

        P = (
            f"\n _________________________________________________________\n"
            f"| ------------------ GPS stochastic Info ---------------- |\n"
            f"| GPS position sigma x:           "+"{:.6f}".format(self.sigposx)         +" [m]            |\n"
            f"| GPS position sigma y:           "+"{:.6f}".format(self.sigposy)         +" [m]            |\n"
            f"| GPS position sigma z:           "+"{:.6f}".format(self.sigposz)         +" [m]            |\n"
            f"|_________________________________________________________|\n"
        )

        print(P)


                
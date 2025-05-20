import json
import logging
import coloredlogs

# Create a logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG) 

coloredlogs.install(level='DEBUG', logger=logger)

class GPSPitchHeadingStochasticParameter:
    def __init__(self):
        
        # sigma position [m]
        self.sigpitch: float = 0
        self.sigheading: float = 0

    def setsigpitch(self, sigpitch ):
        self.sigpitch = sigpitch

    def setsigheading(self, sigheading ):
        self.sigheading = sigheading

    # ##################################################################################
    # Write function

    def writetojson(self, path, filename):

        #p("Write IMU config file: ", path + filename)
        logger.info("Writing .json GPS pitch heading configfile")

        data = {
                "sigpitch": self.sigpitch,
                "sigheading": self.sigheading
                }

        # Write the data to a JSON file
        with open(path + filename, 'w') as json_file:
            json.dump(data, json_file, indent=4)

    def readfromjson(self, path, filename):
        # Construct the full path to the JSON file
        filepath = path + filename

        logger.info( "Reading .json GPS pitch heading configfile: " + path + filename )
        
        # Open and read the JSON file
        with open(filepath, 'r') as json_file:
            data = json.load(json_file)

        # Access specific elements
        self.sigpitch = data["sigpitch"]
        self.sigheading = data["sigheading"]

        P = (
            f"\n _________________________________________________________\n"
            f"| --------- GPS pitch heading stochastic Info ----------- |\n"
            f"| GPS pitch sigma:             "+"{:.6f}".format(self.sigpitch)         +" [rad]             |\n"
            f"| GPS heading sigma:           "+"{:.6f}".format(self.sigheading)         +" [rad]             |\n"
            f"|_________________________________________________________|\n"
        )

        print(P)
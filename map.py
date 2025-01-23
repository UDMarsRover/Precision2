from PyQt6.QtWidgets import *
from PyQt6.QtGui import QPixmap, QImage
from PIL import Image, ImageDraw


class Map(QWidget):
    def __init__(self):
        super().__init__()

        self.image_label = QLabel()

        self.pin_window = QPushButton("Open Pins")
        self.pin_window.clicked.connect(self.open_pin_window)
        self.pin_button = QPushButton("Place Pin")
        self.pin_button.clicked.connect(self.place_pin)

        self.layout = QGridLayout()
        self.layout.addWidget(self.image_label, 0,0, 3,3)
        self.layout.addWidget(self.pin_button, 3,0)
        self.layout.addWidget(self.pin_window, 3,1)


        self.setLayout(self.layout)
        
        self.pin_locations = []

        self.set_image()

        self.show()
    
    def set_image(self):
        #This loads the image (Obviously change the path to where it is in the Pi)
        path = r"C:\Users\weave\Documents\Programming\PythonScripts\KLTestImage.jpg"
        #This is the path to the map with the pins overlayed
        pin_path = r"C:\Users\weave\Documents\Programming\PythonScripts\KLTestImage_withPins.jpg"

        image = Image.open(path)
        draw = ImageDraw.Draw(image)

        #Place pins at the stored locations
        pin_radius = 10
        for lat, lon in self.pin_locations:
            x, y = self.to_pixels(lat, lon)
            print(x, y)
            draw.circle((x,y), pin_radius, "red")
        #This saves the image so it can be re-opened by the QImage class
        image.save(pin_path)

        #Re-opens the image and sets the windows image to that file
        image = QImage(pin_path)

        #Update these to make it fit
        self.new_width = 600
        self.new_height = 450
        scaled_image = image.scaled(self.new_width, self.new_height)

        pixmap = QPixmap.fromImage(scaled_image)
        self.image_label.setPixmap(pixmap)
        self.show()
    
    def to_pixels(self, lat, lon):
        #The coordinates for the upper left and bottom righthand corners of the map
        self.coordsUL = self.dms_to_decimal("39°44'13.39\"N"), self.dms_to_decimal("84°10'42.79\"W")
        self.coordsBR = self.dms_to_decimal("39°44'19.77\"N"), self.dms_to_decimal("84°10'28.04\"W")

        #This assumes a 1080p resolution
        self.widthP = 1920
        self.heightP = 1080

        #sets up a ratio of how far in the x direction the pin is in relation to the screen
        #Then, mutliplying by the screen width and height gives you the pixel coordinates
        self.yPixel = self.heightP * (lat - self.coordsUL[0]) / (self.coordsBR[0] - self.coordsUL[0])
        self.xPixel = self.widthP * (self.coordsUL[1]- lon) / (self.coordsUL[1] - self.coordsBR[1])

        print(self.xPixel, self.yPixel)

        return int(self.xPixel), int(self.yPixel)
    
    def place_pin(self):
        #add lat and long coords input
        self.lat =  "39°44'16.79\"N"
        self.lon = "84°10'35.00\"W"

        self.lat = self.dms_to_decimal(self.lat)
        self.lon = self.dms_to_decimal(self.lon)

        self.pin_locations.append((self.lat, self.lon))
        self.set_image()

        #How to actually add? Keep list of locations, auto add (timer), manual add (button)
        #just pins with numerical labels?

    def open_pin_window(self):
        self.window = QWidget()

        self.window.layout = QVBoxLayout()
        self.window.text_out = QTextEdit()

        self.window.text_out.setReadOnly(True)

        self.window.layout.addWidget(self.window.text_out)

        self.window.setLayout(self.window.layout)

        self.window.text_out.append("Latitude and Longitude in Decimal")    #Very easy to change to DMS if needed
        for x,y in self.pin_locations:
            self.window.text_out.append("(" + str(x) + ", " + str(y) + ")")
        
        self.window.show()

    def dms_to_decimal(self, dms_str):
        # Split the string by degree, minute, and second symbols
        dms_str.strip()
        direction = dms_str[-1]
        dms_str = dms_str[:-1]  # Remove the direction character (E, W, N, S)
        
        # Separate degrees, minutes, and seconds
        degrees, minutes, seconds = 0, 0, 0
        if '°' in dms_str:
            degrees = float(dms_str.split('°')[0])
            minutes_seconds = dms_str.split('°')[1]
            if "'" in minutes_seconds:
                minutes = float(minutes_seconds.split("'")[0])
                if '"' in minutes_seconds:
                    seconds = float(minutes_seconds.split("'")[1].replace('"', ''))
        
        # Convert to decimal
        decimal_degrees = degrees + (minutes / 60) + (seconds / 3600)
        
        # Apply negative sign for W and S directions
        if direction in ['W', 'S']:
            decimal_degrees = -decimal_degrees
        
        return decimal_degrees
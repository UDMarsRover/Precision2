import sys
import cv2
from PyQt6.QtWidgets import *
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt, QProcess, QTimer, QCommandLineOption, QCommandLineParser
import map
import windows

class Button(QPushButton):

    def __init__(self, name, color, connection): #Add size?
        super().__init__()

        self.setText(name)
        self.color = color
    
        #Connects the string of the method name to the attributed method
        clickMethod = getattr(self, connection)
        self.clicked.connect(clickMethod)

        #Creates a stylesheet to assign a color to the widget
        self.defaultSheet = ("QPushButton {"
            "background-color: " + color + ";"  # Background color
            "color: white;"               # Text color
            "}"
            "QPushButton:hover {"   #Convert name to lighter shade somehow. Webcolors name_to_rgb?
            "background-color: silver;"  # Hover color
            "color: black"
            "}")

        self.litSheet = ("QPushButton {"
                "background-color: orange;"
                "color: black;"               
                "}"
                "QPushButton:hover {"
                "background-color: silver;"
                "color: black"
                "}")

        self.setStyleSheet(
            self.defaultSheet
        )

        self.setMaximumWidth(100)
        self.show()
    
    def exitClicked(FakeArgument1, fakeArgument2):
        global closer
        closer = exitWindow()

    def confirm_exit(self, fakeArgument1):  
        app.closeAllWindows()
        terminal.process.close()
        
    def cancel_exit(self, fakeArgument1):
        closer.close()

    def lightsOut(self, fakeArgument1):
        global visableLights
        global infraredLights
        visableLights = False
        infraredLights = False
        lightsVis.setStyleSheet(
            lightsVis.defaultSheet
        )
        lightsNV.setStyleSheet(
            lightsNV.defaultSheet
        )

    def lightsVis(self, fakeArgument1):
        global visableLights
        if visableLights:
            visableLights = False
            lightsVis.setStyleSheet(
                lightsVis.defaultSheet
            )
        else:
            visableLights = True
            lightsVis.setStyleSheet(
                lightsVis.litSheet
            )
            

    def lightsNV(self, fakeArgument1):
        global infraredLights
        if infraredLights:
            infraredLights = False
            lightsNV.setStyleSheet(
                lightsNV.defaultSheet
            )
        else:
            infraredLights = True
            lightsNV.setStyleSheet(
                lightsNV.litSheet
            )

    def lightsAllOn(self, fakeArgument1):
        global visableLights
        visableLights = True
        global infraredLights
        infraredLights = True
        lightsVis.setStyleSheet(
            lightsVis.litSheet
        )
        lightsNV.setStyleSheet(
            lightsNV.litSheet
        )
    
    def take_screenshot(self):
        pass #Do this when Josh figures out how signals work

    def high_res_camera(self):
        global cameraMode
        global highResCam
        global infraRedCam
        if cameraMode == "Infrared":
            highResCam.setStyleSheet("QPushButton {"
                "background-color: blue;"
                "color: black;"               
                "}"
                "QPushButton:hover {"
                "background-color: silver;"
                "color: black"
                "}")
            infraRedCam.setStyleSheet(self.defaultSheet)
            cameraMode = "High Res"

    def infrared_camera(self):
        global cameraMode
        global highResCam
        global infraRedCam
        if cameraMode == "High Res":
            infraRedCam.setStyleSheet("QPushButton {"
                "background-color: blue;"
                "color: black;"               
                "}"
                "QPushButton:hover {"
                "background-color: silver;"
                "color: black"
                "}")
            highResCam.setStyleSheet(self.defaultSheet)
            cameraMode = "Infrared"
   
class exitWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()

        label = QLabel("Are you sure you want to exit?")
        yes_button = Button("Exit", "red", "confirm_exit")
        no_button = Button("Cancel", "green", "cancel_exit")

        self.layout.addWidget(label)
        self.layout.addWidget(yes_button)
        self.layout.addWidget(no_button)

        self.setLayout(self.layout)
        self.show()

class Camera(QWidget):
    def __init__(self):
        super().__init__()
        self.image_label = QLabel()
        
        self.sliderH = QSlider(Qt.Orientation.Horizontal)
        self.sliderV = QSlider(Qt.Orientation.Vertical)
        self.screenshot = Button("Take Screenshot", "navy", "take_screenshot")

        #Creates camera mode buttons
        global cameraMode
        cameraMode = "High Res"
        global highResCam
        highResCam = Button("High Res", "gray", "high_res_camera")
        highResCam.setStyleSheet("QPushButton {"       #Sets the high res camera button to blue (active)
            "background-color: blue;"
            "color: black;"               
            "}"
            "QPushButton:hover {"
            "background-color: silver;"
            "color: black"
            "}")
        global infraRedCam
        infraRedCam = Button("Infrared", "gray", "infrared_camera")

        self.sliderH.setRange(0,100)
        self.sliderV.setRange(0,100)
        
        self.button_container = QWidget()
        self.button_container.layout = QHBoxLayout()
        self.button_container.layout.addWidget(infraRedCam)
        self.button_container.layout.addWidget(highResCam)
        self.button_container.layout.addWidget(self.screenshot)
        self.button_container.setLayout(self.button_container.layout)

        #Creates a layout and applies the widgets
        self.layout = QGridLayout()
        self.layout.addWidget(self.sliderH, 0,0)
        self.layout.addWidget(self.button_container, 4,0)
        self.layout.addWidget(self.image_label, 1,0, 3,3)
        self.layout.addWidget(self.sliderV, 3,3)
        self.setLayout(self.layout)
        
        self.cap = cv2.VideoCapture(0)

        #Creates a timer that will update the webcam feed every 30 milliseconds
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(30)
        
    def update(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format.Format_RGB888)

            #Scales the image so it fits better
            self.new_width = 600
            self.new_height = 450
            scaled_image = image.scaled(self.new_width, self.new_height)

            # Convert the QImage to a QPixmap so PyQt6 can read it
            pixmap = QPixmap.fromImage(scaled_image)
            
            
            self.image_label.setPixmap(pixmap)
            
            self.show()
        
        #Is it better to add a segment here so the slide bars update based 
        #on time intervals, or update only when their value is changed?
    """
    Fix me when you know how these values will be read
    
    def update_bars(self):
        #These are all the variables the angle bars would need from the camera gimble
        #This assumes the left and right
        self.maxH = 100
        self.angleH = 50
        self.maxV = 100
        self.angleV = 50
        
        self.positionH = 2* self.maxH / self.angleH
        self.sliderH.setSliderPosition(self.positionH)

        self.positionH = 2* self.maxH / self.angleH
        self.sliderH.setSliderPosition(self.positionH)
        """      

"""
class Terminal(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Terminal")
        self.terminalOut = QTextEdit()
        self.terminalOut.setReadOnly(True)

        self.terminalIn = QLineEdit()
        self.terminalIn.returnPressed.connect(self.execute_command)  # Run command on Enter

        #Adds everything to the layouts
        layout = QVBoxLayout()
        layout.addWidget(self.terminalOut)
        layout.addWidget(self.terminalIn)
        self.setLayout(layout)

        #Starts a process to auto-update the output terminal
        self.process = QProcess()
        self.process.readyReadStandardOutput.connect(self.handle_output)
        self.process.start("cmd.exe")  # Or "bash" on linux

        self.environment = {}

    def handle_output(self): #Updates the output
        command = "ls" if sys.platform != "win32" else "dir"
        self.output_area.append(f"Executing: {command}")
        self.process.start(command)

    
    def execute_command(self):
        # Get the text from the input field
        command = self.terminalIn.text().strip()
        
        # Display the command in the output area
        self.terminalOut.append(f">>> {command}")
        
        # Try executing the command
        try:
            # Compile and execute code within the local environment
            result = eval(command, self.environment)
            if result is not None:
                self.terminalOut.append(str(result))
        except Exception as e:
            self.terminalOut.append(f"Error: {e}")
        
        # Clear the input field
        self.terminalIn.clear() 

    def parse(self, app):
        parser = QCommandLineParser()

        parser.addHelpOption()
        parser.addVersionOption()

        parser.process(app)

"""
class Output_Window(QWidget):
    #Setup subscriber here ***
    def __init__(self):
        super().__init__()

        self.text_out = QTextEdit()
        self.text_out.setReadOnly(True)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.text_out)
        self.setLayout(self.layout)

        self.show()
    
    def output(self, message):
        self.text_out.append(message)

if __name__ == "__main__":
    global app      #Global so it can be closed in "exit_confirmed"
    app = QApplication(sys.argv)

    #Creating main widgets
    window = windows.mainWindow()
    camera = Camera()
    global terminal     #Global so it can be closed in "exit_confirmed"
    #terminal = Terminal()
    mapObj = map.Map()
    output_window = Output_Window()

    #Buttons!
    lightsButtonList = []
    
    exitButton = Button("Exit?", "red", "exitClicked")

    lightsOut = Button("Lights Off", "gray", "lightsOut")
    lightsButtonList.append(lightsOut)

    global lightsVis
    lightsVis = Button("Visible Lights", "gray", "lightsVis")
    lightsButtonList.append(lightsVis)

    global lightsNV
    lightsNV = Button("Infrared Lights", "gray", "lightsNV")
    lightsButtonList.append(lightsNV)

    global lightsAllOn
    lightsAllOn = Button("All Lights On", "gray", "lightsAllOn")
    lightsButtonList.append(lightsAllOn)

    #These are the variables for the lights states. 
    #Depending on how signaling is easiest, maybe just replace with on/off signals
    global visableLights
    visableLights = False
    global infraredLights
    infraredLights = False

    #Series of containers that store the buttons int the upper right hand dock widget
    layout = QHBoxLayout()
    for i in lightsButtonList:
        layout.addWidget(i)
    lightsContainer = QWidget()
    lightsContainer.setLayout(layout)

    #terminal.parse(app)

    #Adding all the widgets to the window
    window.layout.addWidget(camera, 0,0, 2,2)
    window.layout.addWidget(lightsContainer, 0,2)
    #window.layout.addWidget(terminal, 2,2)
    window.layout.addWidget(exitButton, 0,5)
    window.layout.addWidget(mapObj, 2,0)
    window.layout.addWidget(output_window, 1,2)

    app.exec()
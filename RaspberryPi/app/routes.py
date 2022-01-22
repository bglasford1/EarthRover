#
# Copyright 2020, William Glasford
#
#  This file contains the main code that runs based on the URL.
#
# Mods:     02/05/20  Initial Release.

import serial
import os
import time
from flask import render_template, flash, redirect, url_for, Response, request, json
from app import app
from app.forms import IndexForm, CommandForm, TestForm, ShutdownForm, ImageForm, StatusForm, HelpForm
from picamera import PiCamera, PiCameraError
from RPLCD.i2c import CharLCD
import netifaces as ni
import numpy as np

# Global variables.
ser = None
lcd = None
ip = None
arduinoText = ""
lastCommandResponse = ""
arduinoCommandString = ""
lastImageName = ""
lastCommands = [' ', ' ', ' ']
lcdAvailable = False
positionIndex = 0
subIndex = 0
imageArray = np.empty((0, 5), dtype="object")
imageArray = np.append(imageArray, np.array([["","","","",""]]), axis = 0)
lastX = 0
lastY = 0

# Global sensor status variables.
lcdStatus = "DOWN"
roboclawAStatus = "DOWN"
roboclawBStatus = "DOWN"
orientationStatus = "DOWN"
lidarStatus = "DOWN"
servoDriverStatus = "DOWN"
frontDropoffStatus = "DOWN"
rearDropoffStatus = "DOWN"

# Global situational awareness variables.
compassHeading = ""
tiltX = ""
tiltY = ""
tiltZ = ""
frontDropoff = ""
rearDropoff = ""
powerVoltage = ""
logicVoltage = ""
solarVoltage = ""
orientationTemp = ""
roboclawATemp = ""
roboclawBTemp = ""

mapDataX = [0]
mapDataY = [0]

lidarData = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
lidarLabels = ['0', '10', '20', '30', '40', '50', '60', '70', '80', '90', '100', '110', '120',
               '130', '140', '150', '160', '170', '180', '190', '200', '210', '220', '230', '240',
               '250', '260', '270', '280', '290', '300', '310', '320', '330', '340', '350']

cameraStatus = "OK"
try:
    camera = PiCamera()
except PiCameraError:
    cameraStatus = "DOWN"

#------------------
# This runs one on first http request.  It opens the Arduino interface and tests the interface.
#------------------
@app.before_first_request
def before_first_request_func():
    connect_to_arduino()

#------------------
# Internal routine to write to the LCD.  This isolates the call.
#------------------
def lcd_write(col, row, data):
    global lcdAvailable

    if lcdAvailable:
        try:
            lcd.cursor_pos = (col, row)
            lcd.write_string(data)
        except OSError:
            pass

#------------------
# Internal routine to initiate the interface to the arduino.
#------------------
def connect_to_arduino():
    global arduinoText
    global ser
    global lcd
    global ip
    global lcdAvailable

    try:
        lcd = CharLCD(i2c_expander='MCP23008', address=0x20, port=1, cols=20, rows=4, backlight_enabled=True)
        lcdAvailable = True
    except OSError:
        flash("LCD not connected!!!")

    lcd_write(0, 0, 'Rover - Initializing')

    ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']

    lcd_write(1, 0, 'IP Addr:' + ip)

    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
        ser.flush()

        # Read data from the Arduino and add to output text box
        line = "test"
        while len(line) > 0:
            line = ser.readline().decode('utf-8')
            arduinoText = arduinoText + line
    except serial.SerialException:
        flash("Arduino not connected!!!")

#------------------
# The home page is used to show general status.
#------------------
@app.route('/')
@app.route('/index')
def index():
    global ser
    global lcd
    global lcdStatus
    global cameraStatus
    global roboclawAStatus
    global roboclawBStatus
    global orientationStatus
    global lidarStatus
    global servoDriverStatus
    global frontDropoffStatus
    global rearDropoffStatus

    form = IndexForm()

    form.lcdStatus.data = lcdStatus
    form.roboclawAStatus.data = roboclawAStatus
    form.roboclawBStatus.data = roboclawBStatus
    form.orientationStatus.data = orientationStatus
    form.lidarStatus.data = lidarStatus
    form.servoDriverStatus.data = servoDriverStatus
    form.frontDropoffStatus.data = frontDropoffStatus
    form.rearDropoffStatus.data = rearDropoffStatus
    form.cameraStatus.data = cameraStatus

    lcd_write(2, 0, 'LABOSLFRA')
    lcd_write(3, 0, '---------')

    if type(ser) is not type(None):
        # Clean out the Arduino serial port queue.  IS THIS REALLY NEEDED???
        line = "test"
        while len(line) > 0:
            line = ser.readline().decode('utf-8')

        # Send the read status command.
        ser.write(bytes("3", 'utf-8'))
        status = int(get_arduino_response())
        lcd_write(3, 8, '+')

        # Parse status byte.
        if status >= 128:
            form.rearDropoffStatus.data = "OK"
            rearDropoffStatus = "OK"
            lcd_write(3, 7, '+')
            status = status - 128

        if status >= 64:
            form.frontDropoffStatus.data = "OK"
            frontDropoffStatus = "OK"
            lcd_write(3, 6, '+')
            status = status - 64

        if status >= 32:
            form.lidarStatus.data = "OK"
            lidarStatus = "OK"
            lcd_write(3, 5, '+')
            status = status - 32

        if status >= 16:
            form.servoDriverStatus.data = "OK"
            servoDriverStatus = "OK"
            lcd_write(3, 4, '+')
            status = status - 16

        if status >= 8:
            form.orientationStatus.data = "OK"
            orientationStatus = "OK"
            lcd_write(3, 3, '+')
            status = status - 8

        if status >= 4:
            form.roboclawAStatus.data = "OK"
            roboclawAStatus = "OK"
            lcd_write(3, 1, '+')
            status = status - 4

        if status >= 2:
            form.roboclawBStatus.data = "OK"
            roboclawBStatus = "OK"
            lcd_write(3, 2, '+')
            status = status - 2

        if status == 1:
            form.lcdStatus.data = "OK"
            lcd_write(3, 0, '+')
            lcdStatus = "OK"

        lcd_write(0, 0, 'Rover - Initialized ')

    return render_template('index.html', form=form)

#------------------
# The situational awareness page is used to show situational awareness.
#------------------
@app.route('/status')
def status():
    global ser
    global lcd
    global cameraStatus
    global compassHeading
    global tiltX
    global tiltY
    global tiltZ
    global frontDropoff
    global rearDropoff
    global powerVoltage
    global logicVoltage
    global solarVoltage
    global orientationTemp
    global roboclawATemp
    global roboclawBTemp

    form = StatusForm()

    # Fill in situational awareness values.
    form.direction.data = compassHeading
    form.tiltX.data = tiltX
    form.tiltY.data = tiltY
    form.tiltZ.data = tiltZ
    form.frontDropDistance.data = frontDropoff
    form.rearDropDistance.data = rearDropoff
    form.drivePower.data = powerVoltage
    form.logicPower.data = logicVoltage
    form.solarPower.data = solarVoltage
    form.dofTemp.data = orientationTemp
    form.roboATemp.data = roboclawATemp
    form.roboBTemp.data = roboclawBTemp

    return render_template('aware.html', form=form)

#------------------
# The Map page is used to map where the rover has traveled.
#------------------
@app.route('/map')
def map():
    global imageArray

    # Convert mapData points into data chartjs expects.
    newList = []
    for X, Y in zip(mapDataX, mapDataY):
        newList.append({'x': X, 'y': Y})
    mapData = str(newList).replace('\'', '')

    # Convert imageArray into JSON that the script expects.
    x = 0
    y = 0
    encodedData = "{ "
    for xval in imageArray:
        encodedData = encodedData + "\"" + str(x) + "element\": { "
        for yval in xval:
            if y == 0 and len(yval) == 0:
                break
            if len(yval) != 0:
                encodedData = encodedData + " \"" + str(y) + "\": \"" + yval + "\","
            y = y + 1
        encodedData = encodedData[:-1] # remove last character
        encodedData = encodedData + "},"
        x = x + 1
        y = 0
    encodedData = encodedData[:-1] # remove last character
    encodedData = encodedData + "}"

    return render_template('map.html', data=mapData, images=encodedData)

#------------------
# The Lidar page is used to display the lidar map.
#------------------
@app.route('/lidar')
def lidar_map():
    return render_template('lidar.html', values=lidarData, labels=lidarLabels)

#------------------
# The Images page is used to manage static images from the camera.
#------------------
@app.route('/images', methods=['GET', 'POST'])
def manage_image():
    global lastImageName

    path = '/srv/MarsRover/app/static'
    files = os.listdir(path)

    form = ImageForm()

    # A button was pressed.
    if request.method == 'POST':

        # Set file to view.
        if form.view.data:
            selected_files = request.form.getlist('files')
            lastImageName = selected_files[0]
            return redirect('/camera')

        # Delete selected files.
        if form.delete.data:
            selected_files = request.form.getlist('files')
            for file in selected_files:
                os.remove(path + '/' + file)
            return redirect('/images')

    return render_template('images.html', form=form, data=files)

#------------------
# The camera page is used to view static images from the camera.
# The image to view is set on the images page.
#------------------
@app.route('/camera', methods=['GET', 'POST'])
def capture_image():
    global lastImageName

    return render_template('camera.html', name=lastImageName)

#------------------
# The restart page is used to restart the robot, specifically to re-initialize the interface to the
#   arduino.
#------------------
@app.route('/restart', methods=['GET', 'POST'])
def restart():
    connect_to_arduino()
    return redirect(url_for('index'))

#------------------
# The shutdown page is used to perform an orderly shutdown of the Raspberry Pi.
#------------------
@app.route('/shutdown', methods=['GET', 'POST'])
def shutdown():
    form = ShutdownForm()

    if form.validate_on_submit():

        # Cancel the action and return to index page.
        if form.cancel.data:
            return redirect(url_for('index'))

        # Shutdown the Raspberry Pi.
        if form.shutdown.data:
            os.system("shutdown now -h")

    return render_template('shutdown.html', form=form)

#------------------
# The command page is used to send a sequence of commands to the Arduino.
#------------------
@app.route('/command', methods=['GET', 'POST'])
def command():
    global arduinoText
    global ser
    global arduinoCommandString
    global lastCommandResponse
    global lastCommands
    global lastImageName
    global compassHeading
    global tiltX
    global tiltY
    global tiltZ
    global frontDropoff
    global rearDropoff
    global powerVoltage
    global logicVoltage
    global solarVoltage
    global orientationTemp
    global roboclawATemp
    global roboclawBTemp
    global lcdStatus
    global roboclawAStatus
    global roboclawBStatus
    global orientationStatus
    global lidarStatus
    global servoDriverStatus
    global frontDropoffStatus
    global rearDropoffStatus
    global lcdAvailable
    global positionIndex
    global subIndex
    global imageArray
    global mapDataX
    global mapDataY
    global lastX
    global lastY

    commandDelimeter = ","
    subCommandDelimeter = " "

    form = CommandForm()
    form.arduino.data = arduinoText
    form.commandString.data = arduinoCommandString
    form.lastCommandResponse.data = lastCommandResponse

    # Add the command to the command string.
    if form.add.data:

        # Perform data validation.
        command = form.command.data

        # Two data fields expected.
        if command == "MF" or command == "MB":
            if not form.data1.data or not form.data2.data:
                flash("No Data for Command, two data fields expected.")
                return render_template('command.html', form=form)

        # One data field expected.
        elif command == "CR" or command == "CL" or command == "CU" or command == "CD" or \
                command == "LR" or command == "LL" or command == "RR" or command == "RL":
            if not form.data1.data:
                flash("No Data for Command, one data field expected.")
                return render_template('command.html', form=form)
            elif form.data2.data:
                flash("Invalid Data, no second data field expected.")
                return render_template('command.html', form=form)

        # Else no data fields expected.
        else:
            if form.data1.data:
                flash("Invalid Data, no first data field expected.")
                return render_template('command.html', form=form)
            elif form.data2.data:
                flash("Invalid Data, no second data field expected.")
                return render_template('command.html', form=form)

        # Add valid command to command string.
        if form.command.data is not None and len(form.command.data) > 0:
            if len(arduinoCommandString) > 0:
                arduinoCommandString = arduinoCommandString + commandDelimeter

            arduinoCommandString = arduinoCommandString + form.command.data

            if form.data1.data is not None and len(form.data1.data) > 0:
                arduinoCommandString = arduinoCommandString + subCommandDelimeter + form.data1.data

            if form.data2.data is not None and len(form.data2.data) > 0:
                arduinoCommandString = arduinoCommandString + subCommandDelimeter + form.data2.data

        form.commandString.data = arduinoCommandString

    # Clear the command string
    if form.clear.data:
        arduinoCommandString = ""
        form.commandString.data = ""

    # Execute the command string.
    if form.execute.data:

        if lcdAvailable:
            lcd.clear()

        lcd_write(0, 0, arduinoCommandString)
        lcd_write(1, 0, lastCommands[0])
        lcd_write(2, 0, lastCommands[1])
        lcd_write(3, 0, lastCommands[2])

        # Parse the command string.
        commandList = arduinoCommandString.split(commandDelimeter)
        for nextCommand in commandList:
            subCommandList = nextCommand.split(subCommandDelimeter)

            # Convert the higher level command to a sequence of lower level commands.
            # Move Forward:
            if subCommandList[0] == "MF": # and roboclawAStatus == "OK":
                # Read compass to get heading.
                direction = send_arduino_command("9", form)

                # Send the move forward command.
                arduinoCommand = "5" + " " + subCommandList[1] + " " + subCommandList[2]
                send_arduino_command(arduinoCommand, form)

                # Convert from polar to cartesian, add to last point and then fill in new data point for map.
                radians = np.deg2rad(int(direction))
                lastX = lastX + int(subCommandList[1]) * np.cos(radians)
                lastY = lastY + int(subCommandList[1]) * np.sin(radians)
                mapDataX = np.append(mapDataX, np.array([lastX]))
                mapDataY = np.append(mapDataY, np.array([lastY]))

                positionIndex = positionIndex + 1
                subIndex = 0
                imageArray = np.append(imageArray, np.array([["","","","",""]]), axis = 0)

            # Move Backward:
            elif subCommandList[0] == "MB" and roboclawAStatus == "OK":
                # Read compass to get heading.
                direction = send_arduino_command("9", form)

                arduinoCommand = "6" + " " + subCommandList[1] + " " + subCommandList[2]
                send_arduino_command(arduinoCommand, form)

                # Convert from polar to cartesian, add to last point and then fill in new data point for map.
                radians = np.deg2rad(int(direction))
                lastX = lastX + int(subCommandList[1]) * np.cos(radians)
                lastY = lastY + int(subCommandList[1]) * np.sin(radians)
                mapDataX = np.append(mapDataX, np.array([lastX]))
                mapDataY = np.append(mapDataY, np.array([lastY]))

                positionIndex = positionIndex + 1
                subIndex = 0
                imageArray = np.append(imageArray, np.array([["","","","",""]]), axis = 0)

            # Rotate Right:
            elif subCommandList[0] == "RR" and roboclawAStatus == "OK":
                arduinoCommand = "7" + " " + subCommandList[1]
                send_arduino_command(arduinoCommand, form)

            # Rotate Left:
            elif subCommandList[0] == "RL" and roboclawAStatus == "OK":
                arduinoCommand = "8" + " " + subCommandList[1]
                send_arduino_command(arduinoCommand, form)

            # Capture Image:
            elif subCommandList[0] == "CI":
                get_camera_image()

            # Center Camera:
            elif subCommandList[0] == "CC":
                send_arduino_command("12", form)

            # Pan Camera Right: first center camera then pan right degrees.
            elif subCommandList[0] == "CR":
                send_arduino_command("12", form)
                arduinoCommand = "10" + subCommandList[1]
                send_arduino_command(arduinoCommand, form)

            # Pan Camera Left: first center camera then pan left degrees.
            elif subCommandList[0] == "CL":
                send_arduino_command("12", form)
                arduinoCommand = "10" + subCommandList[1]
                send_arduino_command(arduinoCommand, form)

            # Tilt Camera Up:
            elif subCommandList[0] == "CU":
                arduinoCommand = "11" + subCommandList[1]
                send_arduino_command(arduinoCommand, form)

            # Tilt Camera Down:
            elif subCommandList[0] == "CD":
                arduinoCommand = "11" + subCommandList[1]
                send_arduino_command(arduinoCommand, form)

            # Read Compass Data:
            elif subCommandList[0] == "OD":
                compassHeading = send_arduino_command("9", form)

            # Read Tilt Data:
            elif subCommandList[0] == "TD":
                tiltX = send_arduino_command("21", form)
                tiltY = send_arduino_command("22", form)
                tiltZ = send_arduino_command("23", form)

            # Read Drop-off Data.
            elif subCommandList[0] == "DD":
                frontDropoff = send_arduino_command("19", form)
                rearDropoff = send_arduino_command("20", form)

            # Read Power Data: read power, logic and solar voltages.
            elif subCommandList[0] == "PD":
                logicVoltage = send_arduino_command("17", form)
                solarVoltage = send_arduino_command("18", form)
                if roboclawAStatus == "OK":
                    powerVoltage = send_arduino_command("16", form)

            # Read Temperature Data: read Orientation, Roboclaw-A and Roboclaw-B temperatures.
            elif subCommandList[0] == "TE":
                orientationTemp = send_arduino_command("24", form)
                roboclawATemp = send_arduino_command("25", form)
                roboclawBTemp = send_arduino_command("26", form)

            # Read Situational Awareness to fill in index page.
            elif subCommandList[0] == "SA":
                # Center Camera
                send_arduino_command("12", form)

                # Center Lidar
                send_arduino_command("14", form)

                # Get other situational awareness data
                compassHeading = send_arduino_command("9", form)
                tiltX = send_arduino_command("21", form)
                tiltY = send_arduino_command("22", form)
                tiltZ = send_arduino_command("23", form)
                frontDropoff = send_arduino_command("19", form)
                rearDropoff = send_arduino_command("20", form)
                orientationTemp = send_arduino_command("24", form)
                roboclawATemp = send_arduino_command("25", form)
                roboclawBTemp = send_arduino_command("26", form)
                logicVoltage = send_arduino_command("17", form)
                solarVoltage = send_arduino_command("18", form)
                if roboclawAStatus == "OK":
                    powerVoltage = send_arduino_command("16", form)

            # Read Situational Imaging to get Lidar map and camera image.
            elif subCommandList[0] == "SA":
                get_camera_image()
                read_lidar_map(form)

            # Read Lidar Map:
            elif subCommandList[0] == "LM":
                read_lidar_map(form)

        lcd_write(0, 0, arduinoCommandString + " - " + lastCommandResponse)

        lastCommands[2] = lastCommands[1]
        lastCommands[1] = lastCommands[0]
        lastCommands[0] = arduinoCommandString + " - " + lastCommandResponse

    return render_template('command.html', form=form)

#------------------
# The test page is used to send low level commands to the Arduino.  This is not normally used
# to control the rover.
#------------------
@app.route('/test', methods=['GET', 'POST'])
def test():
    global arduinoText
    global ser
    global lastCommandResponse
    global lastCommands
    global lcdAvialable

    form = TestForm()
    form.arduino.data = arduinoText
    form.lastCommandResponse.data = lastCommandResponse

    commandToSend = form.command.data
    if form.data1.data is not None and len(form.data1.data) > 0:
        commandToSend = commandToSend + " " + form.data1.data + " "
    if form.data2.data is not None and len(form.data2.data) > 0:
        commandToSend = commandToSend + form.data2.data + " "

    if form.validate_on_submit():
        if lcdAvailable:
            lcd.clear()

        lcd_write(0, 0, commandToSend)
        lcd_write(1, 0, lastCommands[0])
        lcd_write(2, 0, lastCommands[1])
        lcd_write(3, 0, lastCommands[2])

        send_arduino_command(commandToSend, form)

        lcd_write(0, 0, arduinoCommandString + " - " + lastCommandResponse)

        lastCommands[2] = lastCommands[1]
        lastCommands[1] = lastCommands[0]
        lastCommands[0] = commandToSend + " - " + lastCommandResponse

    return render_template('test.html', form=form)

#------------------
# The help page is used to display help text to the user.
#------------------
@app.route('/help', methods=['GET', 'POST'])
def help():
    basedir = os.path.abspath(os.path.dirname(__file__))
    helptext_file = os.path.join(basedir, 'static/helpfile.txt')
    text = ""
    with open(helptext_file, "r") as file:
        for line in file.readlines():
            text = text + line

    form = HelpForm()
    form.helpText.data = text
    return render_template('help.html', form=form)

#------------------
# This handles the page not found error, i.e. an invalid url was submitted.
#------------------
@app.errorhandler(404)
def page_not_found(e):
    return render_template('404.html'), 404

#------------------
# This handles the internal server error.
#------------------
@app.errorhandler(500)
def internal_server_error(e):
    return render_template('500.html'), 500

#------------------
# This internal function gets an Arduino response.
#------------------
def get_arduino_response():
    global arduinoText
    global ser

    line = ser.readline().decode('utf-8')
    while len(line) == 0:
        line = ser.readline().decode('utf-8')

    arduinoText = arduinoText + line
    return line

#------------------
# This internal function sends an Arduino command.
#------------------
def send_arduino_command(command, form):
    global arduinoText
    global ser
    global lastCommandResponse

    ser.write(bytes(command, 'utf-8'))
    lastCommandResponse = get_arduino_response()
    form.arduino.data = arduinoText
    form.lastCommandResponse.data = lastCommandResponse
    return lastCommandResponse

#------------------
# This internal function captures a camera image.
#------------------
def get_camera_image():
    global positionIndex
    global subIndex

    camera.resolution = (1024, 768)
    camera.start_preview()
    time.sleep(2)
    lastImageName = "image" + str(time.time()) + ".jpg"
    camera.capture("app/static/" + lastImageName)

    # Save off image name for map display.
    imageArray[positionIndex, subIndex] = lastImageName
    subIndex = subIndex + 1

#------------------
# This internal function capture lidar data.
#------------------
def read_lidar_map(form):
    # TODO: narrow degrees to at least 5 degrees.
    # Loop from 0 to 180 in 10 degree increments.
    # Translate: 0 -> 270, 80 -> 350, 90 -> 0, 180 -> 90
    for nextPoint in range(0, 19):
        degree = nextPoint * 10
        command = "13 " + str(degree) + " "
        send_arduino_command(command, form)
        send_arduino_command("15", form)
        index = nextPoint + 27
        if (index >= 36):
            index = index - 36
        lidarData[index] = lastCommandResponse
        flash(str(degree) + " degrees : distance = " + lastCommandResponse + " cm : index = " + str(index))
        time.sleep(1)

    # Center when done.
    send_arduino_command("14", form)

#
# Copyright 2020, William Glasford
#
#  This file defines the web page forms.
#
# Mods:     02/05/20  Initial Release.

from flask_wtf import FlaskForm
from wtforms import SelectField, StringField, SubmitField, TextAreaField

class ImageForm(FlaskForm):
    delete = SubmitField('Delete')
    view = SubmitField('View')

class TestForm(FlaskForm):
    command = SelectField('Command:',
                          choices=[('2', 'Test'),
                                   ('3', 'Read Status'),
                                   ('5', 'Move Forward'),
                                   ('6', 'Move Backward'),
                                   ('7', 'Rotate Right'),
                                   ('8', 'Rotate Left'),
                                   ('9', 'Read Compass'),
                                   ('10', 'Pan Camera'),
                                   ('11', 'Tilt Camera'),
                                   ('12', 'Center Camera'),
                                   ('13', 'Pan Lidar'),
                                   ('14', 'Center Lidar'),
                                   ('15', 'Read Lidar'),
                                   ('16', 'Read Power Voltage'),
                                   ('17', 'Read Logic Voltage'),
                                   ('18', 'Read Solar Voltage'),
                                   ('19', 'Read Front Drop-off'),
                                   ('20', 'Read Rear Drop-off'),
                                   ('21', 'Read Tilt X'),
                                   ('22', 'Read Tilt Y'),
                                   ('23', 'Read Tilt Z'),
                                   ('24', 'Read Temp. Orient.'),
                                   ('25', 'Read Temp. Robo-A'),
                                   ('26', 'Read Temp. Robo-B')])
    data1 = StringField('Data 1:')
    data2 = StringField('Data 2:')
    submit = SubmitField('Execute')
    lastCommandResponse = StringField('Last Command Response:')
    arduino = TextAreaField('Arduino Output:')

class CommandForm(FlaskForm):
    command = SelectField('Command:',
                          choices=[('MF', 'Move Forward'),
                                   ('MB', 'Move Backward'),
                                   ('RR', 'Rotate Right'),
                                   ('RL', 'Rotate Left'),
                                   ('SA', 'Situational Awareness'),
                                   ('LM', 'Lidar Map'),
                                   ('SI', 'Situational Imaging'),
                                   ('CC', 'Center Camera'),
                                   ('CI', 'Capture Image'),
                                   ('CR', 'Camera Right'),
                                   ('CL', 'Camera Left'),
                                   ('CU', 'Camera Up'),
                                   ('CD', 'Camera Down'),
                                   ('RC', 'Compass Data'),
                                   ('TD', 'Tilt Data'),
                                   ('DD', 'Drop-off Data'),
                                   ('TE', 'Temperature Data'),
                                   ('PD', 'Power Data')])
    data1 = StringField('Data 1:')
    data2 = StringField('Data 2:')
    add = SubmitField('Add Command')
    clear = SubmitField('Clear Command')
    commandString = StringField('Command String:')
    execute = SubmitField('Execute')
    lastCommandResponse = StringField('Last Command Response:')
    arduino = TextAreaField('Arduino Output:')

class IndexForm(FlaskForm):
    lcdStatus = StringField('Lcd Status:')
    roboclawAStatus = StringField('Roboclaw A Status:')
    roboclawBStatus = StringField('Roboclaw B Status:')
    orientationStatus = StringField('Orientation Sensor Status:')
    servoDriverStatus = StringField('Servo Driver Status:')
    lidarStatus = StringField('Lidar Status:')
    frontDropoffStatus = StringField('Front Drop-off Status:')
    rearDropoffStatus = StringField('Rear Drop-off Status:')
    cameraStatus = StringField('Camera:')

class StatusForm(FlaskForm):
    direction = StringField('Direction:')
    tiltX = StringField('Tilt - X:')
    tiltY = StringField('Tilt - Y:')
    tiltZ = StringField('Tilt - Z:')
    frontDistance = StringField('Front Distance:')
    frontDropDistance = StringField('Front Drop Distance:')
    rearDropDistance = StringField('Rear Drop Distance:')
    solarPower = StringField('Solar Power:')
    logicPower = StringField('Logic Power:')
    drivePower = StringField('Drive Power:')
    roboATemp = StringField('Roboclaw A Temp:')
    roboBTemp = StringField('Roboclaw B Temp:')
    dofTemp = StringField('9 DOF Sensor Temp:')

class HelpForm(FlaskForm):
    helpText = TextAreaField('Help Text:')

class RestartForm(FlaskForm):
    restart = SubmitField('Restart')

class ShutdownForm(FlaskForm):
    shutdown = SubmitField('Shutdown')
    cancel = SubmitField('Cancel')

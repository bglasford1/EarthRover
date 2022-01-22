/*
 * Copyright 2020, William Glasford
 *
 *  Mars Rover:
 *
 *  This is the Arduino code to run the Mars Rover.  The main loop waits on the UART 
 *  for a command from the Raspberry Pi.  Once received, the command is executed and 
 *  a response is sent back.
 *
 *
 * Mods:     02/05/20  Initial Release.
 *  
 */
#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include <LIDARLite.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>
#include <Adafruit_LiquidCrystal.h>

// Pin # definitions
#define ROBOCLAW_TX_PIN 11
#define ROBOCLAW_RX_PIN 10
#define SONAR_TRIGGER_PIN 9
#define SONAR_ECHO_PIN 8
#define FRONT_TRIGGER_PIN 7
#define FRONT_ECHO_PIN 6
#define REAR_TRIGGER_PIN 5
#define REAR_ECHO_PIN 4

// Sonar Definitions.
#define MAX_DISTANCE 200

// Number of A to D samples to take
#define NUM_SAMPLES 10

// LCD Status Positions
#define LCD_STATUS_COL 0
#define ROBOCLAW_STATUS_A_COL 1
#define ROBOCLAW_STATUS_B_COL 2
#define ORIENTATION_SENSOR_STATUS_COL 3
#define SERVO_DRIVER_STATUS_COL 4
#define LIDAR_STATUS_COL 5
#define FRONT_SONAR_STATUS_COL 6
#define REAR_SONAR_STATUS_COL 7
#define RASPBERRYPI_STATUS_COL 8

// Servo Definitions.
#define SERVOMIN  700 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2400 // This is the 'maximum' pulse length count (out of 4096)
#define FREQUENCY 50

// Roboclaw Addresses
#define ROBOCLAW_A_ADDRESS 0x80
#define ROBOCLAW_B_ADDRESS 0x81

// Status Values
bool lcd_status = true;
bool roboclaw_a_status = true;
bool roboclaw_b_status = true;
bool orientation_status = true;
bool servo_driver_status = true;
bool lidar_status = true;
bool front_sonar_status = true;
bool rear_sonar_status = true;

// Define the various objects.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); 

Adafruit_LiquidCrystal lcd(0);

NewPing frontSonar(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing rearSonar(REAR_TRIGGER_PIN, REAR_ECHO_PIN, MAX_DISTANCE);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

LIDARLite lidar;
int lidar_count = 0;

SoftwareSerial serial(ROBOCLAW_RX_PIN, ROBOCLAW_TX_PIN);
RoboClaw roboclaw(&serial, 10000);

// Next command to execute, from the Raspberry PI
String commandString;

// Last 4 commands
String lastCommand1 = "";
String lastCommand2 = "";
String lastCommand3 = "";

// Return Strings
String success = "Success";
String invalidData = "Invalid data";
String noData = "No data";
String invalidCommand = "Invalid command";

/*
 * This is the main Arduino method called once during initialization.  
 */
void setup() 
{
  Serial.begin(9600);
  Serial.println();
  Serial.println("Mars Rover: Initializing...");
  
  // Initialize the LCD 
  lcd.begin(20,4);
  
  lcd.print("Rover - Initializing");

  lcd.setCursor(0,2);
  lcd.print("LABOSLFRI");

  lcd.setCursor(0,3);
  lcd.print("+-------+");

  // Initialize Lidar
  lidar.begin(0, true);
  lidar.configure(0);

  lcd.setCursor(LIDAR_STATUS_COL, 3);
  lcd.print("+");

  // Initialize Pan and Tilt Servos.
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  uint8_t prescale = pwm.readPrescale();
  Serial.println("prescale: " + String(prescale));
  // TODO: instead of println, set board status to down...

  // Initialize the Front Drop-off Sensor
  long distance = readFrontSonar();
  if (distance == 0)
  {
    Serial.println("Front Drop-off Sensor Not Connected!!!");
    front_sonar_status = false;
  }
  else
  {
    lcd.setCursor(FRONT_SONAR_STATUS_COL, 3);
    lcd.print("+");
  }
  
  // Initialize the Rear Drop-off Sensor
  distance = readRearSonar();
  if (distance == 0)
  {
    Serial.println("Rear Drop-off Sensor Not Connected!!!");
    rear_sonar_status = false;
  }
  else
  {
    lcd.setCursor(FRONT_SONAR_STATUS_COL, 3);
    lcd.print("+");
  }

  // Initialize the Orientation Sensor
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    orientation_status = false;
  }
  if (orientation_status)
  {
    bno.setExtCrystalUse(true);
    lcd.setCursor(ORIENTATION_SENSOR_STATUS_COL, 3);
    lcd.print("+");
  }

  // Initialize Roboclaws
  roboclaw.begin(38400);

  bool valid;
  uint16_t errorA = roboclaw.ReadError(ROBOCLAW_A_ADDRESS, &valid);
  if (!valid)
  {
    Serial.println("Roboclaw A not connected!");
    roboclaw_a_status = false;
  }
  else
  {
    lcd.setCursor(ROBOCLAW_STATUS_A_COL, 3);
    lcd.print("+");
  }

  uint16_t errorB = roboclaw.ReadError(ROBOCLAW_B_ADDRESS, &valid);
  if (!valid)
  {
    Serial.println("Roboclaw B not connected!");
    roboclaw_b_status = false;
  }
  else
  {
    lcd.setCursor(ROBOCLAW_STATUS_B_COL, 3);
    lcd.print("+");
  }

  Serial.println("Mars Rover: Started.");
  lcd.setCursor(0, 0);
  lcd.print("Rover - Initialized ");
}

/*
 * This is the main Arduino loop that runs continuously.  It waits for a command from the 
 * Raspberry Pi, then executes the command before waiting for the next command.  The commands are:
 * 5  = Move Forward, distance in inches, speed (% of total)
 * 6  = Move Backwards, distance in inches, speed (% of total)
 * 7  = Rotate Right, degrees to rotate
 * 8  = Rotate Left, degrees to rotate
 * 9  = Read Compass
 * 10 = Pan Camera, degrees to rotate
 * 11 = Tilt Camera, degrees to tilt
 * 12 = Center Camera
 * 13 = Pan Lidar, degrees to rotate
 * 14 = Center Lidar
 * 15 = Read Lidar
 * 16 = Read Power Voltage
 * 17 = Read Logic Voltage
 * 18 = Read Solar Voltage
 * 19 = Read Front Drop-off Sensor
 * 20 = Read Rear Drop-off Sensor
 * 21 = Read Tilt X
 * 22 = Read Tilt Y
 * 23 = Read Tilt Z
 * 24 = Read Temperature of Orientation Sensor
 * 25 = Read Temperature of Roboclaw A
 * 26 = Read Temperature of Roboclaw B
 * 
 * NOTE: When running from the Serial Monitor, connected to the PC, you have to add a character to the 
 * command string.  Ex: 5 12 33x
 */
void loop() 
{
   // Read the next command sequence.
  while (Serial.available()) 
  {
    delay(2);  // Delay 2 milliseconds to allow byte to arrive in input buffer
    char c = Serial.read();  // Gets one byte from serial buffer
    
    if (c != '\n')  
      commandString += c; 
  }

  // If there is a command string to parse...
  if (commandString.length() > 0) 
  {
    // Clear the display and write the current command and last 3 commands.
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(commandString);
    lcd.setCursor(0, 1);
    lcd.print(lastCommand1);
    lcd.setCursor(0, 2);
    lcd.print(lastCommand2);
    lcd.setCursor(0, 3);
    lcd.print(lastCommand3);
    
    // Parse the command string.
    String command = "";
    String data1 = "";
    String data2 = "";
    String delim = " ";
    int firstDelimIndex = commandString.indexOf(delim);
    int secondDelimIndex = commandString.indexOf(delim, firstDelimIndex + 1);

    // Strip off any line feed if exists.
    
    // If only a command number then need to calculate end of command string.
    if (firstDelimIndex == -1)
    {
      // Note: monitor sends CR/LF where as Raspberry Pi does not.
      command = commandString.substring(0, commandString.length());
    }
    else // Use the first delimeter index.
    {
      command = commandString.substring(0, firstDelimIndex);
      if (secondDelimIndex == -1)
      {
        data1 = commandString.substring(firstDelimIndex + 1, commandString.length() - 1);
      }
      else
      {
        data1 = commandString.substring(firstDelimIndex + 1, secondDelimIndex);
        data2 = commandString.substring(secondDelimIndex + 1, commandString.length() - 1);
      }
    }

    // Process the Test command.
    if (command == "2")
    {
      displayData("Test");
    }

    // Process the Read Status command.
    else if (command == "3")
    {
      int statusInt = 0;
      if (lcd_status)
        statusInt += 1;
      if (roboclaw_a_status)
        statusInt += 2;
      if (roboclaw_b_status)
        statusInt += 4;
      if (orientation_status)
        statusInt += 8;
      if (servo_driver_status)
        statusInt += 16;
      if (lidar_status)
        statusInt += 32;
      if (front_sonar_status)
        statusInt += 64;
      if (rear_sonar_status)
        statusInt += 128;

      displayData(String(statusInt));
    }

    // Process the Move Forward command.
    else if (command == "5")
    {
      if (data1.length() > 0 && data2.length() > 0)
      {
        moveRover(data1, data2, true, true);
        displayData(success);
      }
      else
      {
        displayData(invalidData);
      }
    }

    // Process the Move Backward command.
    else if (command == "6")
    {
      if (data1.length() > 0 && data2.length() > 0)
      {
        moveRover(data1, data2, false, false);
        displayData(success);
      }
      else
      {
        displayData(invalidData);
      }
    }

    // Process the Rotate Right command.
    else if (command == "7")
    {
     if (data1.length() > 0 && data2.length() > 0)
      {
        moveRover(data1, data2, false, true);
        displayData(success);
      }
      else
      {
        displayData(invalidData);
      }
    }

    // Process the Rotate Left command.
    else if (command == "8")
    {
     if (data1.length() > 0 && data2.length() > 0)
      {
        moveRover(data1, data2, true, false);
        displayData(success);
      }
      else
      {
        displayData(invalidData);
      }
    }

    // Process the Get Compass command.
    else if (command == "9")
    {
      double yaw = readCompass();
      displayData(String(yaw, 3));
    }

    /*
     * The pan and tilt servos are numbered: 
     * 0 = camera pan, 1 = camera tilt, 2 = lidar pan, 3 = lidar tilt
     * The Pan direction: 90 = straight ahead, 0 is left, 180 is right.
     * The Tilt direction: 90 = straight, 180 is forward, 0 is back (55 is as far as it should go)
     */
     
    // Process the Pan Camera command.
    else if (command == "10")
    {
      if (data1.length() == 0)
      {
        displayData(noData);
      }
      else
      {
        int degree = data1.toInt();
        if (degree > 180 || degree < 0)
        {
          displayData(invalidData);
        }
        else
        {
          pwm.setPWM(0, 0, pulseWidth(degree));
        displayData(success);
        }
      }
    }

    // Process the Tilt Camera command.
    else if (command == "11")
    {
      if (data1.length() == 0)
      {
        displayData(noData);
      }
      else
      {
        int degree = data1.toInt();
        if (degree > 120 || degree < 0)
        {
          displayData(invalidData);
        }
        else
        {
          pwm.setPWM(1, 0, pulseWidth(degree));
          displayData(success);
        }
      }
    }

    // Process the Center Camera command.
    else if (command == "12")
    {
      pwm.setPWM(0, 0, pulseWidth(90));
      pwm.setPWM(1, 0, pulseWidth(90));
      displayData(success);
    }

    // Process the Pan Lidar command.
    else if (command == "13")
    {
       if (data1.length() == 0)
      {
        displayData(noData);
      }
      else
      {
        int degree = data1.toInt();
        if (degree > 180 || degree < 0)
        {
          displayData(invalidData);
        }
        else
        {
          pwm.setPWM(2, 0, pulseWidth(degree));
          displayData(success);
        }
      }
    }

    // Process the Center Lidar command.
    else if (command == "14")
    {
      pwm.setPWM(2, 0, pulseWidth(90));
      displayData(success);
    }

    // Process the Read Lidar command.
    else if (command == "15")
    {
      int distance = readLidar();
      displayData(String(distance));
    }

    // Process the Read Power Voltage command, from the Roboclaw A.
    else if (command == "16")
    {
      bool valid;
      uint16_t voltage = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_A_ADDRESS, &valid);
      float powerVoltage = (float)voltage / 10;
      if (valid)
      {
        displayData(String(powerVoltage));
      }
    }

    // Process the Read Logic Voltage command.
    else if (command == "17")
    {
      float voltage = readLogicVoltage();
      displayData(String(voltage));
    }

    // Process the Read Solar Voltage command.
    else if (command == "18")
    {
      // TODO: Implement
      displayData("Not Implemented!!!");
    }

    // Process the Front Front Drop-off Sensor command
    else if (command == "19")
    {
      long distance = readFrontSonar();
      displayData(String(distance));
    }

    // Process the Read Rear Drop-off Sensor command
    else if (command == "20")
    {
      long distance = readRearSonar();
      displayData(String(distance));
    }

    // Process the Get Tilt X command
    else if (command == "21")
    {
      float tiltX = readTiltX();
      displayData(String(tiltX, 3));
    }

    // Process the Get Tilt Y command
    else if (command == "22")
    {
      float tiltY = readTiltY();
      displayData(String(tiltY, 3));
    }

    // Process the Get Tilt Z command
    else if (command == "23")
    {
      float tiltZ = readTiltZ();
      displayData(String(tiltZ, 3));
    }

    // Process the Get Temperature of Orientation sensor command
    else if (command == "24")
    {
      int8_t degFtemp = readTemperature();
      displayData(String(degFtemp));
    }
    
    // Process the Get Temperature of Roboclaw A command
    else if (command == "25")
    {
      uint16_t temperature; // Temperature  in tenths.
      roboclaw.ReadTemp(ROBOCLAW_A_ADDRESS, temperature);
      float tempC = (float)temperature / 10;
      float tempF = tempC * 1.8 + 32;
      displayData(String(tempF));
    }
 
    // Process the Get Temperature of Roboclaw A command
    else if (command == "26")
    {
      uint16_t temperature; // Temperature in tenths.
      roboclaw.ReadTemp(ROBOCLAW_B_ADDRESS, temperature);
      float tempC = (float)temperature / 10;
      float tempF = tempC * 1.8 + 32;
      displayData(String(tempF));
    }
    else
    {
      displayData(invalidCommand);
    }
    
    lastCommand3 = lastCommand2;
    lastCommand2 = lastCommand1;
    lastCommand1 = commandString;
    commandString = "";
  } 
}

/*
 * Internal method to display status of command to both the LCD and back to the Raspberry PI.
 */
void displayData(String statusValue)
{
  commandString = commandString + " - " + statusValue;
  Serial.println(statusValue);
  lcd.setCursor(0, 0);
  lcd.print(commandString);
}

/*
 * Move rover forward by activating all 4 motors with same speed, measuring distance using
 * whichever motor is moving forward.  If both motors are moving backwards then the counting
 * loop is reverse logic.  The speed is converted from percent of 100 to 0 - 127 where 0 is 
 * full stop and 127 is full speed.  The distance is converted to number of encoder counts
 * per inch.  
 * 
 * Distance: distance in inches
 * Speed: percent of total speed
 * RightDirection: true = forward, false = backward
 * LeftDirection: true = forward, false = backward
 */
void moveRover(String distance, String speedPercent, boolean rightDir, boolean leftDir)
{
  // Zero encoders so they can count the distance.
  boolean test0 = roboclaw.ResetEncoders(ROBOCLAW_A_ADDRESS);
  boolean test1 = roboclaw.ResetEncoders(ROBOCLAW_B_ADDRESS);

  float speed = speedPercent.toFloat() / 100 * 128;
  int maxCount = distance.toInt() * 100;

  // If both motors are going backwards then need to set encoder count to max and let it count down to zero
  if (rightDir && !leftDir)
  {
    roboclaw.SetEncM1(ROBOCLAW_A_ADDRESS, maxCount);
  }

  // The while loop is positive logic if the motor is running forward and negative is in reverse.
  // The right side is negative logic, i.e. forward is backwards.
  engageMotors((int)speed, rightDir, leftDir);

  uint32_t encoderCount;
  // Moving Forward or Rotating Right
  if ((rightDir && leftDir) || (!rightDir && leftDir))
  {
    do
    {
      encoderCount = roboclaw.ReadEncM1(ROBOCLAW_B_ADDRESS);
    } while (encoderCount < maxCount);  //Loop until distance command has completed
  }
  // Moving Backwards
  else if (!rightDir && !leftDir)
  {
    do
    {
      encoderCount = roboclaw.ReadEncM1(ROBOCLAW_A_ADDRESS);
    } while (encoderCount < maxCount);  //Loop until distance command has completed
  }
  // Rotating Left
  else
  {
    do
    {
      encoderCount = roboclaw.ReadEncM1(ROBOCLAW_A_ADDRESS);
    } while (encoderCount < maxCount + 1);  //Loop until distance command has completed
  }
  
  // Now stop the motors.
  engageMotors(0, rightDir, leftDir);
}

/*
 * Method to move the rover either forward or backward.  Note: the right motor is reverse logic.
 * 
 * Speed = internal speed value; 0 = stop, 127 = max speed
 * rightDir: true = forward, false = backward
 * leftDir: true = forward, false = backward
 */
void engageMotors(int speed, boolean rightDir, boolean leftDir)
{
  if (rightDir)
  {
    roboclaw.BackwardM1(ROBOCLAW_A_ADDRESS, speed);
    roboclaw.BackwardM2(ROBOCLAW_A_ADDRESS, speed);
  }
  else
  {
    roboclaw.ForwardM1(ROBOCLAW_A_ADDRESS, speed);
    roboclaw.ForwardM2(ROBOCLAW_A_ADDRESS, speed);
  }

  if (leftDir)
  {
    roboclaw.ForwardM1(ROBOCLAW_B_ADDRESS, speed);
    roboclaw.ForwardM2(ROBOCLAW_B_ADDRESS, speed);
  }
  else
  {
    roboclaw.BackwardM1(ROBOCLAW_B_ADDRESS, speed);
    roboclaw.BackwardM2(ROBOCLAW_B_ADDRESS, speed);
  }
}

/*
 * Read the logic voltage which is connected to the A2 pin. 
 */
float readLogicVoltage()
{
  int sum = 0;                    // sum of samples taken
  unsigned char sample_count = 0; // current sample number
  float voltage = 0.0;            // calculated voltage
  
  // take a number of analog samples and add them up
  while (sample_count < NUM_SAMPLES) 
  {
    sum += analogRead(A2);
    sample_count++;
    delay(10);
  }
  
  // calculate the voltage, 5.14V is the measured reference voltage
  voltage = ((float)sum / (float)NUM_SAMPLES * 5.14) / 1024.0;
  
  // voltage is multiplied by 11.177 which is the calibrated voltage divide value
  return voltage * 11.177;
}

/*
 * Read the absolute magnetometer readings (x, y, z) on the BNO055 sensor and convert the readings
 * to a yaw value and then to a 0-360 degree reading.
 */
double readCompass()
{
  sensors_event_t magEvent;
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  if (magEvent.magnetic.x == 0.0 && magEvent.magnetic.y == 0.0)
  {
    delay(100);
    bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  }

  double magX = -1000000, magY = -1000000 , magZ = -1000000;
   magX = magEvent.magnetic.x;
   magY = magEvent.magnetic.y;
   magZ = magEvent.magnetic.z;

  double yaw = atan2(magY, magX) * 180/3.14159;

  if (yaw < 0)
  {
    yaw = 180 + (180 + yaw);
  }
  return yaw;
}

/*
 * Read the absolute orientation values on the BNO055 sensor and return the X value.
 */
float readTiltX()
{
  sensors_event_t event; 
  bno.getEvent(&event);
  if (event.orientation.x == 0.0)
  {
    delay(100);
    bno.getEvent(&event);
  }
  return event.orientation.x;
}

/*
 * Read the absolute orientation values on the BNO055 sensor and return the Y value.
 */
float readTiltY()
{
  sensors_event_t event; 
  bno.getEvent(&event);
  if (event.orientation.y == 0.0)
  {
    delay(100);
    bno.getEvent(&event);
  }
  return event.orientation.y;
}

/*
 * Read the absolute orientation values on the BNO055 sensor and return the Z value.
 */
float readTiltZ()
{
  sensors_event_t event; 
  bno.getEvent(&event);
  if (event.orientation.z == 0.0)
  {
    delay(100);
    bno.getEvent(&event);
  }
  return event.orientation.z;
}

/*
 * Read the temperature value on the BNO055 sensor.
 */
int8_t readTemperature()
{
  int8_t temp = bno.getTemp();
  int8_t degFtemp = temp * 9 / 5 + 32;
  return degFtemp;
}

/*
 * Read the Front Drop-Off Sensor and return the distance in centimeters.
 */
long readFrontSonar()
{
  long distance = frontSonar.ping_median(20);
  return frontSonar.convert_cm(distance);
}

/*
 * Read the Rear Drop-Off Sensor and return the distance in centimeters.
 */
long readRearSonar()
{
  long distance = rearSonar.ping_median(20);
  return rearSonar.convert_cm(distance);
}

/*
 * Read the Lidar Sensor and return the distance in centimeters.
 */
int readLidar()
{
  // At beginning of every 100 readings, take measurement w/ bias.
  int dist;
  if (lidar_count == 0)
  {
    dist = lidar.distance(); // With bias correction
  }
  else
  {
    dist = lidar.distance(false); // Without bias
  }

  lidar_count++;
  lidar_count = lidar_count % 100;
  
  return dist;
}

/*
 * Method to convert an angle in degrees into a servo analog PWM value.
 */
int pulseWidth(int angle)
{
  int pulse_width, analog_value;
  pulse_width = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  analog_value = int(float(pulse_width) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

/*

arduino thermocycle serial command handling

to start the program this is the command

START

to stop the program this is the command

STOP

to preheat the heat block this is the command with a default temperature of 90c

PRE_HEAT

to set preheat temperature this is the command

PRE_HEAT T=95

to cooldown the heat block this is the command

COOLDOWN

to get the PID value this is the command

GET_PID

to set the PID tune value this is the command

PID_TUNE SET_PID P=2 I=1 D=1

to set the PID tune temperature this is the command

PID_TUNE T=95

to get the list of program this is the command

GET_PROGRAMS

to set the program this is the command

SET_PROGRAM S1 T=95 D=50 RR=1

to set the program cycle this is the command

SET_PROGRAM CYCLES=90

*/

// Include necessary libraries
#include <SerialCommand.h>
#include <PID_v1.h>
#include <ThermocycleStep.h>
#include <EEPROM.h>
#include <ThermocyclerDisplay.h>

#define THERMISTOR_PIN A2           // Pin connected to thermistor
#define REF_RESISTOR 10000          // Resistance of the reference resistor in ohms
#define ROOM_TEMP_RESISTANCE 100000 // Resistance of the thermistor at room temperature in ohms

// Initialize SerialCommand instance
SerialCommand sCmd;

// Define the number of samples to use in the moving average filter
const int numSamples = 10;

// Define an array to store the previous temperature samples
double samples[10];

// Define pins for the motor driver
const int motorPin1 = 6;
const int motorPin2 = 5;
const int enablePin1 = 3;
const int enablePin2 = 4;

// Define pin for the heater
const int heaterPin = 10;

// Define power pins
const int vcc[] = {2, 15, 12}; // List of VCC pin numbers
const int gnd[] = {7, 14};     // List of GND pin numbers

ThermocyclerDisplay thermocyclerDisplay;

// Define the PID parameters
double Setpoint, Input, Output;
double Kp = 9.4, Ki = 0.11, Kd = 8.2; // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define enums for the program and thermal states
enum ProgramState
{
  Idle,
  Running,
  Stopped,
  PID_Tune,
  ErrorProgram
};

enum ThermalState
{
  Heating,
  Cooling,
  Holding,
  ErrorThermal
};

bool isDataLogging = false;

// Define program and thermal state variables
float tolerance = 1.5;
bool equilibrating = false;
bool preHeating = false;
float preHeatTemp = 90;
ProgramState programState = Idle;
ThermalState thermalState;

// Define cycle count and number of cycles
int cycleCount = 0;
int numCycles = 2;

// Define the thermocycler program as a sequence of ThermocycleStep objects
ThermocycleStep program[] = {
    //(in °C, in seconds, ramp rate)
    ThermocycleStep("Denaturation", 95, 30, 0), // Denaturation
    ThermocycleStep("Annealing", 55, 30, 0),    // Annealing
    ThermocycleStep("Extension", 72, 45, 0),    // Extension
    ThermocycleStep("Final", 72, 600, 0),       // Final Extension and Cooling
};

// Define variables for the thermocycler program
unsigned int currentStep = 0;
unsigned long startTime = 0;
unsigned long readTemperatureTimer = 0; // Timer for temperature
unsigned long serialTimer = 0;          // Timer for serial communication
unsigned long logRate = 250;            // For serial communication
ThermocycleStep currentThermocycleStep = program[currentStep];

void getPID()
{
  // Send current PID values over serial
  Serial.print(F("Current PID values: Kp = "));
  Serial.print(Kp);
  Serial.print(F(", Ki = "));
  Serial.print(Ki);
  Serial.print(F(", Kd = "));
  Serial.println(Kd);
}

float getTemperature()
{
  int reading = analogRead(THERMISTOR_PIN);                                                                  // Read analog input from thermistor pin
  float resistance = REF_RESISTOR * (1023.0 / reading - 1.0);                                                // Calculate thermistor resistance using voltage divider formula
  float temperature = 1.0 / (1.0 / 298.15 + 1.0 / 3977.0 * log(resistance / ROOM_TEMP_RESISTANCE)) - 273.15; // Calculate temperature using Steinhart-Hart equation
  return temperature;
}

void setPID()
{
  String param = sCmd.next();
  // extract and set P, I, and D values
  while (param != NULL)
  {
    if (param.startsWith("P="))
    {
      Kp = param.substring(2).toDouble();
      // Set P value in PID controller
    }
    else if (param.startsWith("I="))
    {
      Serial.println(param);
      Ki = param.substring(2).toDouble();
      // Set I value in PID controller
    }
    else if (param.startsWith("D="))
    {
      Kd = param.substring(2).toDouble();
      // Set D value in PID controller
    }
    myPID.SetTunings(Kp, Ki, Kd);
    getPID();
    param = sCmd.next();
  }
}

// Function to load PID values from EEPROM
void loadPIDValues(double &savedKp, double &savedKi, double &savedKd, int &flag)
{
  // Load the PID values from EEPROM if they have been saved before
  int addr = 0;
  EEPROM.get(addr, savedKp);
  addr += sizeof(savedKp);
  EEPROM.get(addr, savedKi);
  addr += sizeof(savedKi);
  EEPROM.get(addr, savedKd);
  addr += sizeof(savedKd);
  EEPROM.get(addr, flag);
}

void initPIDValue()
{

  double savedKp, savedKi, savedKd;
  int flag;

  loadPIDValues(savedKp, savedKi, savedKd, flag);

  if (flag == 1)
  {
    // If the flag is set, load the saved values and update the PID controller
    Kp = savedKp;
    Ki = savedKi;
    Kd = savedKd;

    Serial.println(F("PID values loaded from EEPROM."));
  }
  else
  {
    // If the flag is not set, use the default values and save them to EEPROM
    EEPROM.put(0, Kp);
    EEPROM.put(sizeof(Kp), Ki);
    EEPROM.put(sizeof(Kp) + sizeof(Ki), Kd);
    EEPROM.put(sizeof(Kp) + sizeof(Ki) + sizeof(Kd), 1);

    Serial.println(F("Default PID values saved to EEPROM."));
  }
}

// PIDTune function to handle PID tuning.
void PIDTune()
{
  programState = PID_Tune;    // set program state to PID tuning
  String param = sCmd.next(); // get the next parameter from serial monitor

  // loop through all the parameters
  while (param != NULL)
  {
    // set the PID values
    if (param.startsWith("SET_PID"))
    {
      setPID(); // call setPID function
    }
    // exit PID tuning mode
    else if (param.startsWith("DONE"))
    {
      programState = Idle;                   // set program state to Idle
      Serial.println(F("PID tuning done!")); // print message to serial monitor

      // Stop the motor and turn off the motor driver
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin1, 0);
      analogWrite(enablePin2, 0);

      break; // exit the loop
    }
    // save the PID values to EEPROM
    else if (param.startsWith("SAVE_PID"))
    {
      double savedKp, savedKi, savedKd;
      int flag;

      loadPIDValues(savedKp, savedKi, savedKd, flag); // load PID values from EEPROM

      // check if there are any changes made to the PID values
      if (savedKp == Kp && savedKi == Ki && savedKd == Kd)
      {
        Serial.println(F("No changes made")); // print message to serial monitor
        break;                                // exit the loop
      }
      // save the PID values to EEPROM
      int addr = 0;
      EEPROM.put(addr, Kp);
      addr += sizeof(Kp);
      EEPROM.put(addr, Ki);
      addr += sizeof(Ki);
      EEPROM.put(addr, Kd);
      addr += sizeof(Kd);
      EEPROM.put(addr, 1); // save a flag to indicate that the values have been saved

      // send a response back to the serial monitor
      Serial.println(F("PID values updated and saved to EEPROM."));
    }
    // set the target value
    else if (param.startsWith("T="))
    {
      Setpoint = param.substring(2).toDouble();              // get the target value and convert it to double
      Serial.println("Target sets to: " + String(Setpoint)); // print message to serial monitor
    }
    param = sCmd.next(); // get the next parameter from serial monitor
  }
}

void preHeat()
{
  // Check if the program is not already running
  if (programState != Running)
  {
    String param = sCmd.next(); // get the next parameter from serial monitor

    if (!preHeating)
    {
      preHeating = true;
      Setpoint = preHeatTemp;
    }

    if (param != NULL)
    {
      // set the target value
      if (param.startsWith("T="))
      {
        Setpoint = param.substring(2).toDouble();              // get the target value and convert it to double
        Serial.println("Target sets to: " + String(Setpoint)); // print message to serial monitor
      }
    }
  }
  else
  {
    Serial.println(F("Info: Preheating a running program is not possible."));
  }
}

void cooldown()
{
  // Check if the program is not already running
  if (programState != Running && programState == Idle)
  {
    if (preHeating)
    {
      preHeating = false;
      Setpoint = 0;

      // Stop the motor and turn off the motor driver
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin1, 0);
      analogWrite(enablePin2, 0);

      Serial.println(F("Cooling down!")); // print message to serial monitor
    }
  }
  else
  {
    Serial.println(F("Info: Cooling down a running program is not possible."));
  }
}

int digitCount(int num)
{
  int count = 0;
  while (num > 0)
  {
    count++;
    num /= 10;
  }
  return count;
}

void getPrograms()
{
  Serial.println(F("\nThermocycle program:\n"));
  Serial.println(F("|Step Name       |Temperature (C)|Duration (s)|Ramp Rate|"));

  for (int i = 0; i < sizeof(program) / sizeof(program[0]); i++)
  {

    ThermocycleStep step = program[i];
    // Get the duration of the current step
    unsigned long duration = step.getDuration();

    Serial.print(F("|"));
    Serial.print(step.getName());
    Serial.print(F(" "));
    for (int j = step.getName().length(); j < 15; j++)
    {
      Serial.print(F(" "));
    }

    Serial.print(F("|"));
    Serial.print(step.getTemperature());
    Serial.print(F(" "));
    for (int j = digitCount(step.getTemperature()); j < 11; j++)
    {
      Serial.print(F(" "));
    }

    Serial.print(F("|"));
    Serial.print(duration);
    Serial.print(F(" "));
    for (int j = digitCount(duration); j < 11; j++)
    {
      Serial.print(F(" "));
    }

    Serial.print(F("|"));
    Serial.print(step.getRampRate());
    Serial.print(F(" "));
    for (int j = digitCount(step.getRampRate()); j < 4; j++)
    {
      Serial.print(F(" "));
    }

    Serial.println(F("|"));
  }

  Serial.print(F("No. of cycles: "));
  Serial.println(numCycles);
}

void programReset()
{
  // Reset the setpoint, cycle count, current step, and program state
  Setpoint = 0;
  cycleCount = 0;
  currentStep = 0;
  currentThermocycleStep = program[0];
}

// Function to start the thermocycling program
void startProgram()
{
  // Check if the program is not already running
  if (programState != Running)
  {
    programReset();
    programState = Running;
    preHeating = false;
    startTime = millis();

    Serial.println(F("Program running!"));

    // Display the name and duration of the current step
    Serial.print(F("\n\n\nStarting step "));
    Serial.print(currentStep + 1);
    Serial.print(F(": "));
    Serial.print(currentThermocycleStep.getName());
    Serial.print(F(" for "));
    Serial.print(currentThermocycleStep.getDuration());
    Serial.println(F(" seconds.\n\n\n"));
  }
  else
  {
    Serial.println(F("Program already running"));
  }
}

void setProgram()
{
  String param = sCmd.next(); // get the next parameter from serial monitor

  if (param != NULL)
  {

    if (param.startsWith("CYCLES="))
    {
      numCycles = param.substring(7).toInt();
    }
    else
    {

      int index;

      if (param.startsWith("S1"))
      {
        index = 0;
      }
      if (param.startsWith("S2"))
      {
        index = 1;
      }
      if (param.startsWith("S3"))
      {
        index = 2;
      }
      if (param.startsWith("F"))
      {
        index = 3;
      }

      if (index >= 0 && index < 4)
      {
        ThermocycleStep step = program[index]; // set the program step at the given index

        double temp = step.getTemperature();
        int duration = step.getDuration();
        double rampRate = step.getRampRate();

        param = sCmd.next();

        while (param != NULL)
        {

          if (param.startsWith("T="))
          {
            temp = param.substring(2).toDouble();
          }
          else if (param.startsWith("D="))
          {
            duration = param.substring(2).toDouble();
          }
          else if (param.startsWith("RR="))
          {
            rampRate = param.substring(3).toDouble();
          }
          else
          {
            Serial.println(F("Invalid parameter"));
            return;
          }

          param = sCmd.next();
        }

        step.setDuration(duration);
        step.setTemperature(temp);
        step.setRampRate(rampRate);

        program[index] = step;

        Serial.print(step.getName());
        Serial.println(F(" program set successfully."));
      }
      else
      {
        Serial.println(F("Invalid program index."));
      }
    }
  }
}

void serialDataLog()
{
  isDataLogging = true;

  String param = sCmd.next(); // get the next parameter from serial monitor

  if (param != NULL)
  {
    if (param.startsWith("R="))
    {
      logRate = param.substring(2).toInt();
    }
  }
}

// Function to stop the thermocycling program
void stopProgram()
{
  // Check if the program is running
  if (programState == Running)
  {
    // Stop the motor and turn off the motor driver
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin1, 0);
    analogWrite(enablePin2, 0);

    // Display message on the LCD screen
    thermocyclerDisplay.programStopped();

    // Print message to serial monitor
    Serial.println(F("Program stopped!"));

    // Reset the program state
    programReset();
    programState = Stopped;
  }
  else
  {
    Serial.println(F("Program not running"));
  }
}

// Function to indicate completion of the thermocycling program
void programComplete()
{
  // Check if the program is running
  if (programState == Running)
  {
    // Stop the motor and turn off the motor driver
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin1, 0);
    analogWrite(enablePin2, 0);

    // Display message on the LCD screen
    thermocyclerDisplay.programComplete();

    // Print message to serial monitor
    Serial.println(F("Program complete!"));

    // Reset program state
    programReset();
  }
  else
  {
    Serial.println(F("Program not running"));
  }
}

void readTemperature()
{
  // Read the temperature from the MAX6675 module every 1 second
  if (millis() - readTemperatureTimer >= 250)
  {
    // Add the current temperature to the array of previous samples
    for (int i = 0; i < numSamples - 1; i++)
    {
      samples[i] = samples[i + 1];
    }
    samples[numSamples - 1] = getTemperature();
    ;

    // Compute the moving average temperature
    double sum = 0;
    for (int i = 0; i < numSamples; i++)
    {
      sum += samples[i];
    }

    double movingAverage = sum / numSamples;

    Input = movingAverage;
    readTemperatureTimer = millis();
  }
}

void updateTemperatureControl()
{

  // Execute the PID algorithm to control the motor driver PWM signal
  myPID.Compute();

  // Set the motor driver direction and speed based on the PID output
  if (Output > 0)
  {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, Output);
    thermalState = Heating;
  }
  else if (Output < 0)
  {
    analogWrite(motorPin1, abs(Output));
    analogWrite(motorPin2, 0);
    thermalState = Cooling;
  }
  else
  {
    // Stop the motor and set the thermal state to holding
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    thermalState = Holding;
  }
}

void programRunning()
{

  // Set the target temperature based on the current step of the thermocycle with ramp rate
  Setpoint = currentThermocycleStep.getTemperature() + (currentThermocycleStep.getRampRate() * (millis() - startTime) / 1000);

  // // Set the target temperature based on the current step of the thermocycle
  // Setpoint = currentThermocycleStep.getTemperature();

  // Run the Peltier element to heat/cool the system
  updateTemperatureControl();

  // Check if the temperature is within the tolerance range
  if (abs(Input - Setpoint) <= tolerance)
  {
    // Get the duration of the current step
    unsigned long duration = currentThermocycleStep.getDuration();

    // Display the current temperature and thermocycle information

    thermocyclerDisplay.display(currentThermocycleStep.getName(), cycleCount, Input, duration * 1000, (millis() - startTime));

    // Check if the current thermocycle step is complete
    if (millis() - startTime >= duration * 1000)
    {
      // If the current step is the last step of a cycle, increment the cycle count
      if (currentStep == 2)
      {
        cycleCount++;

        // Check if the desired number of cycles has been completed
        if (cycleCount == numCycles)
        {
          Serial.println(F("Cycles completed."));
          currentStep++;
        }
        else
        {
          // Display the cycle count and reset the step to zero for the next cycle
          Serial.print(F("Cycle "));
          Serial.print(cycleCount);
          Serial.println(F(" completed."));
          currentStep = 0;
        }
      }
      else
      {
        // If the current step is not the last step, increment the step count
        currentStep++;
      }

      // Check if the program is complete
      if (currentStep >= sizeof(program) / sizeof(program[0]))
      {
        programComplete();
      }
      else
      {
        // Set the current thermocycle step and start the timer
        currentThermocycleStep = program[currentStep];
        startTime = millis();

        // Display the name and duration of the current step
        Serial.print(F("\n\n\nStarting step "));
        Serial.print(currentStep + 1);
        Serial.print(F(": "));
        Serial.print(currentThermocycleStep.getName());
        Serial.print(F(" for "));
        Serial.print(currentThermocycleStep.getDuration());
        Serial.println(F(" seconds.\n\n\n"));
      }
    }
  }
  else
  {
    // If the temperature is not within the tolerance range, display the equilibrating temperature
    thermocyclerDisplay.displayEquilibrating(Input, Setpoint);

    startTime = millis();
  }
}

void programIdle()
{
  if (preHeating)
  {
    updateTemperatureControl();
  }
  thermocyclerDisplay.displayIdle(Input, Setpoint, preHeating);
}

void dataSerialLog()
{
  // Print debug information to the serial monitor every 250ms
  if (millis() - serialTimer >= logRate)
  {
    Serial.print(F(" Set_point:"));
    Serial.print(Setpoint);
    Serial.print(F(" Ouput:"));
    Serial.print(Output);
    Serial.print(F(" Input:"));
    Serial.println(Input);
    serialTimer = millis();
  }
}

void programPIDTune()
{
  updateTemperatureControl();
  thermocyclerDisplay.displayTuning(Input, Kp, Ki, Kd);
  dataSerialLog();
}

void setup()
{
  // Initialize the serial communication
  Serial.begin(9600);

  // Initialize the PID saved values
  initPIDValue();

  // Set VCC pins as output and set them HIGH
  for (int i = 0; i < 3; i++)
  {
    pinMode(vcc[i], OUTPUT);
    digitalWrite(vcc[i], HIGH);
  }

  // Set GND pins as output and set them LOW
  for (int i = 0; i < 2; i++)
  {
    pinMode(gnd[i], OUTPUT);
    digitalWrite(gnd[i], LOW);
  }

  // Set the motor driver pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // Set the heater pin as an output
  pinMode(heaterPin, OUTPUT);

  // Turn on the motor driver by setting enable pins HIGH
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);

  // Add serial commands for starting/stopping program, setting/getting PID
  sCmd.addCommand("START", startProgram);
  sCmd.addCommand("STOP", stopProgram);
  sCmd.addCommand("GET_PID", getPID);
  sCmd.addCommand("PID_TUNE", PIDTune);
  sCmd.addCommand("PRE_HEAT", preHeat);
  sCmd.addCommand("COOLDOWN", cooldown);
  sCmd.addCommand("GET_PROGRAMS", getPrograms);
  sCmd.addCommand("SET_PROGRAM", setProgram);
  sCmd.addCommand("PLOTTER", serialDataLog);

  // Initialize LCD display
  thermocyclerDisplay.init();
}

void loop()
{
  // Read serial input and update program state
  sCmd.readSerial(); // Read any incoming serial data and execute commands accordingly

  // Read temperature from sensor
  readTemperature(); // Read the temperature from the sensor and update the temperature variables

  // Check if the program is currently running
  if (programState == Running)
  {
    programRunning(); // Execute the code for running the program (heating/cooling control)
  }

  // If the program is not running (Idle state), display the idle screen
  if (programState == Idle)
  {
    programIdle(); // Execute the code for displaying the idle screen
  }

  // If the program is in PID tuning mode, execute the code for tuning the PID values
  if (programState == PID_Tune)
  {
    programPIDTune(); // Execute the code for PID tuning
  }

  // If data logging is enabled, send data to serial port for logging
  if (isDataLogging)
  {
    dataSerialLog(); // Execute the code for logging data to serial port
  }
}

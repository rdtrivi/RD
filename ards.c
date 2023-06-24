#include <SerialCommand.h>
#include <PID_v1.h>
#include <ThermocycleStep.h>
#include <ThermocyclerDisplay.h>
#define THERMISTOR_PIN A3           
#define REF_RESISTOR 10000          
#define ROOM_TEMP_RESISTANCE 100000 
SerialCommand sCmd;
const int numSamples = 10;
double samples[10];
const int motorPin1 = 6;
const int motorPin2 = 5;
ThermocyclerDisplay thermocyclerDisplay;


double Setpoint, Input, Output; // Define the PID parameters
double Kp = 15, Ki = 0.22, Kd = 0; // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

enum ProgramState 
{
  Idle,
  Running,
};
bool isDataLogging = true;

// Define program and thermal state variables
float tolerance = 2.0;
bool preHeating = false;
ProgramState programState = Idle;

// Define num cycles 
int cycleCount = 0;
int numCycles = 35;

ThermocycleStep program[] = {      // Define the thermocycler program
    //(in °C, in seconds, ramp rate)
    ThermocycleStep("Pre Denaturation, 95,600,0),// Pre Denaturation
    ThermocycleStep("Denaturation", 95, 30, 0), // Denaturation
    ThermocycleStep("Annealing", 55, 30, 0),    // Annealing
    ThermocycleStep("Extension", 72, 30, 0),    // Extension
    ThermocycleStep("Final", 72, 600, 0),       // Final Extension and Cooling
};

// Define variables
unsigned int currentStep = 0;  // Define variables
unsigned long startTime = 0;
unsigned long readTemperatureTimer = 0; // Timer for temperature
unsigned long serialTimer = 0;          // Timer for serial communication
unsigned long logRate = 1000;            // For serial communication
ThermocycleStep currentThermocycleStep = program[currentStep];

float getTemperature()
{
  int reading = analogRead(THERMISTOR_PIN);                                                                  // Read analog input from thermistor pin
  float resistance = REF_RESISTOR * (1023.0 / reading - 1.0);                                                // Calculate thermistor resistance using voltage divider formula
  float temperature = 1.0 / (1.0 / 298.15 + 1.0 / 3977.0 * log(resistance / ROOM_TEMP_RESISTANCE)) - 273.15; // Calculate temperature using Steinhart-Hart equation
  return temperature;
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
}

void startProgram()  // Function to start the thermocycling program
{
  if (programState != Running)
  {
    programState = Running;
    preHeating = false;
    startTime = millis();
    Serial.println(F("Program running!"));
    Serial.print(F("\n\n\nStarting step "));
    Serial.print(currentStep + 1);
    Serial.print(F(": "));
    Serial.print(currentThermocycleStep.getName());
    Serial.print(F(" for "));
    Serial.print(currentThermocycleStep.getDuration());
    Serial.println(F(" seconds.\n\n\n"));
  }
}

void readTemperature()
{
  // Read the temperature from the thermistor every 1 second
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
  myPID.Compute();   // Execute the PID algorithm
     //Heating
   if (Output > 0)
  {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, Output);
  }
  //Cooling
  else if (Output < 0)
  {
    analogWrite(motorPin1, abs(Output));
    analogWrite(motorPin2, 0);
  }
  else
  //Holding
  {
    // Stop the motor and set the thermal state to holding
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
  }
}
void programRunning()
{  
  // // Set the target temperature based on the current step of the thermocycle
   Setpoint = currentThermocycleStep.getTemperature();

  // Run the Peltier element to heat/cool the system
  updateTemperatureControl();
  // Check if the temperature is within the tolerance range
  if (abs(Input - Setpoint) <= tolerance)
  {
    // Get the duration of the current step
    unsigned long duration = currentThermocycleStep.getDuration();
    thermocyclerDisplay.display(currentThermocycleStep.getName(), cycleCount, Input, duration * 1000, (millis() - startTime));
    // Check if the current thermocycle step is complete
    if (millis() - startTime >= duration * 1000)
    {
      // If the current step is the last step of a cycle, increment the cycle count
      if (currentStep == 3)
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
          currentStep = 1;
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


void setup()
{
  // Initialize the serial communication
  Serial.begin(9600);
  // Set the motor driver pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);
  // For starting/stopping program
  sCmd.addCommand("START", startProgram);
  sCmd.addCommand("PRE_HEAT", preHeat);
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
  // If data logging is enabled, send data to serial port for logging
  if (isDataLogging)
  {
    dataSerialLog(); // Execute the code for logging data to serial port
  }
}

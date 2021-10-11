#include "Arduino.h"

//#include "BLDCMotor.h"
//#include "sensors/MagneticSensorI2C.h"
//#include "drivers/BLDCDriver3PWM.h"



/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(MA730_SPI, PA4);
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PA0);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial2);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 4;
  motor.PID_velocity.D = 0.0003;
  motor.LPF_velocity.Tf = 0.05;
  motor.P_angle.P = 20;
  motor.velocity_limit = 50;
  //motor.voltage_limit = 4;

  // velocity low pass filtering time constant
  // the lower the less filtered
  //motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  //motor.P_angle.P = 20;
  // maximal velocity of the position control
  //motor.velocity_limit = 20;

  // use monitoring with serial
  Serial2.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial2);


  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial2.println(F("Motor ready."));
  Serial2.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  //Serial2.println(sensor.getAngle());

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
//Serial2.print(sensor.getAngle());
  //Serial2.print("\t");
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}

/**
  * @brief  Main Function
  * @param  None
  * @retval None
  */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    GPIO_JTAG_Disable();
    //SysClock_Init(F_CPU_128MHz);
    Delay_Init();
    ADCx_Init(ADC1);
    setup();
    for(;;)loop();
}


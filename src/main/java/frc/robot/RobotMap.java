/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.

  // Autonomous constants
  public static int robotLength = 18;
  public static int robotWidth = 33;
  public static int encoderTicksPerInch = 326;

  public static boolean enablePneumatics = false;

  public static double distanceBetweenWheels = 30; // inches (need to be converted to SI units as needed); used in
                                                   // kinematic driving

  /**
   * Starting positions along the field. SI units. The X is a distance from the
   * edge along the long end of the field, and Y is a distance from the edge along
   * the short end distance 5.0, 13.5 in 2020/2021 games mean 5 meters along the
   * long side and in the center along the short end
   */
  public final static double startingPoseX = 5.0;
  public final static double startingPoseY = 13.5;

  // Drivetrain Motor Controllers
  public final static int frontLeftDriveMotorControllerID = 4;
  public final static int backLeftDriveMotorControllerID = 3;
  public final static int frontRightDriveMotorControllerID = 2;
  public final static int backRightDriveMotorControllerID = 1;

  // Intake motor controllers
  public final static int intakeMotorControllerID = 12;
  public final static int loaderFrontMotorControllerID = 23;
  public final static int loaderRearMotorControllerID = 11;

  // Shooter motor controllers
  public final static int shooterWheelMotorControllerID = 10;
  public final static int ShooterTiltMotorControllerID = 31;
  public final static int shooterPanMotorControllerID = 22;
  //If we switch to a falcon motor for shooter
  public final static int shooterWheelFalconMotorControllerID = 5;

  // shooter constants
  /* TODO: get a more accurate value for this */
  //public final static int shooterPanMotorEncoderTicksPerRotation = 3977;
  public  static int shooterPanMotorEncoderTicksPerRotation = 178;
  public final static int shooterXResolution = 640;
  public final static int shooterYResolution = 240; 
  public final static int shooterResolutionAcceptableError = 10;
  public  static double shooterPanSpeed = -.1;
  //public static int shooterPanMotorEncoderFrontVal = 2270;
  //public static double shooterEstimatedPos90PanEncoderVal = 3250;
  //public static double shooterEstimatedNeg90PanEncoderVal = 1300;
  public static int shooterPanMotorEncoderFrontVal = 2816;
  public static double shooterEstimatedPos90PanEncoderVal = 3796;
  public static double shooterEstimatedNeg90PanEncoderVal = 1856;


  public static int shooterTiltMotorTicksPerRotation = 1024;
  public static double shooterTiltMotorTicksPerDegree = 360 / shooterTiltMotorTicksPerRotation; // 360 deg / 1024 ticks
  public static double tiltFangsUpperLimit = 730; //
  public static double tiltFangsLowerLimit = 0; // 
  public static int tiltFangsMiddle = 200;

  public static int minYTiltPixel = 400;
  public static int maxYTiltPixel = 250;

  public static double distMaxTilt = 10;
  public static double distMinTilt = 25;


  // Control panel constants
  // TODO: Get actual motor ID
  public static final int diskSpinnerMotorControllerID = 20;
  //quadrature motor controller ticks per revolution
  public static final int quadratureEncoderTicksPerRev = 178;
  // diameter of the wheel which spins the control panel wheel, in cm
  public static final double diskSpinnerDiameter = 10.16;
  // diameter of the control panel disk in cm
  public static final double controlPanelDiameter = 81.28;
  /**
   * factor to indicate the direction on the motor that the encoder ticks are
   * positive. If clockwise, keep 1; if counterclockwise, change to -1.
   */
  public static final int controlPanelDirectionFactor = -1;

  // Climber constants
  public final static int climberMotorControllerID = 13;

  //TODO: cleanup
  // Driver Input Devices
 
  /*
  //dual stick setup
  public final static int driveStickPort = OI.drivestickPort;
  public final static int buttonBoxPort = OI.copilot;
  public final static int rightJoystickPort = OI.turnStick;
 */

  //single stick setup
  //public final static int drivestickPort = OI.driveStickPort;
  //public final static int turnStickPort = OI.turnStickPort;
  //public final static int buttonBoxPort = OI.copilotPort;
  
  //Deadband values
  public final static double deadbandX = 0.1;
  public final static double deadbandY = 0.1;
  public final static double deadbandZ = 0.1;

  // PCM forward, reverse channels for doubleSolenoids
  // We now have far, FAR fewer solenoids
  public static int IntakeSolenoidForwardChannel = 0;
  public static int IntakeSolenoidReverseChannel = 1;


  public static int falconBotSwitchPortNumber = 0;
  public static boolean isFalconBot;
  public static boolean isSplitStick;

  // How many encoder clicks per revolution (change to 2048 for falcon 500
  // encoders)
  public static int encoderUnitsPerShaftRotation = 4096;
  // The difference between the left and right side encoder values when the robot
  // is rotated 180 degrees
  public static int encoderUnitsPerRobotRotation = 38585;// thats the SUM of the two
  // Allowable error to exit movement methods
  public static int defaultAcceptableError = 1000;
  public static int neckMotor;


  // Closed loop constants
  // How long we wait for a configuration change to happen before we give up and
  // report a failure in milliseconds
  public final static int configureTimeoutMs = 30;
  // Full motor output value
  public final static int fullMotorOutput = 1023;
  // How many milliseconds between each closed loop call
  public final static int closedLoopPeriodMs = 1;
  // Motor neutral dead-band, set to the minimum 0.1%
  public final static double NeutralDeadband = 0.001;

  public final static int Izone_0 = 500;
  public final static double PeakOutput_0 = 1;

  // Closed loop Aux PID parameter values
  public final static double P_1 = 0.75 * fullMotorOutput / encoderUnitsPerShaftRotation; 
  // 75% motor output when error = one rotation
  public final static double I_1 = 0.005 * fullMotorOutput / encoderUnitsPerShaftRotation;
  public final static double D_1 = 0.1;
  public final static double F_1 = 0.227; // just a guesstimate
  public final static int Izone_1 = 500;
  public final static double PeakOutput_1 = 1;

  // Closed loop PAN PID parameter values 
  // Modified for Closed loop position control
  public final static int PID_PAN = 0;
  public final static double P_PAN = 1.5;
  public final static double I_PAN = 0.0002;
  public final static double D_PAN = 15;
  public final static double F_PAN = 0; // set to zero for position closed loop
  // Allowable error to exit movement methods
  public static int panDefaultAcceptableError = 1;
  public static int panAcceleration = 50;
  public static int panCruiseVelocity = 50;
  public final static int panSmoothing = 3;

  // Closed loop TILT PID parameter values 
  // Modified for Closed loop position control
  public final static int PID_TILT = 0;
  public final static double P_TILT = 10;
  public final static double I_TILT = 0.04;
  public final static double D_TILT = 01;
  public final static double F_TILT = 0; // set to zero for position closed loop 


  // Allowable error to exit movement methods
  public static int tiltDefaultAcceptableError = 10;
  
  // MotionMagic curve smoothing parameter [0 - 8]
  public final static int tiltSmoothing =3;

  public static int tiltCruiseVelocity = 50;
  public static int tiltAcceleration = 50;


  // Allowable error to exit vision tracking movement methods
  public static int allowableLeft = ((RobotMap.shooterXResolution / 2) - (RobotMap.shooterResolutionAcceptableError));
  public static int allowableRight = ((RobotMap.shooterXResolution / 2) + (RobotMap.shooterResolutionAcceptableError));
  public static int allowableBelow = ((RobotMap.shooterYResolution / 2) - (RobotMap.shooterResolutionAcceptableError));
  public static int allowableAbove = ((RobotMap.shooterYResolution / 2) + (RobotMap.shooterResolutionAcceptableError));

  public final static double encoderTicksPerDegreeX = 11;  // for Turret Encoder
  public final static double potentiometerTicksPerDegreeY = 1; // TODO check this value
  public final static double pixelsPerDegreeX = 12;  //based on lifecam having a 53.4 degree viewing angle and 640 horizontal pixels
  public final static double pixelsPerDegreeY = 15.2; //480 vertical pixels / 31.6 degree viewing angle

  /**
   * Talon PID methods often demand slot ID's, so we need to do this :(
   */
  public static int SLOT_0 = 0;



  public static void IAmFalconBot() {
    // TODO: Drivetrain dependent values MUST be moved into their respective classes

    // How many encoder clicks per revolution (change to 2048 for falcon 500
    // encoders)
    encoderUnitsPerShaftRotation = 2048;
    // with 6 in wheels estimate 10 feet = 13038 encoder ticks

    // The difference between the left and right side encoder values when the robot
    // is rotated 180 degrees
    encoderUnitsPerRobotRotation = 3925;// thats the SUM of the two (this is just a rough guess)
    // Allowable error to exit movement methods
    defaultAcceptableError = 250;

    shooterPanMotorEncoderTicksPerRotation = 3977;
    //TODO: may need to be negative if turns the wrong way
    shooterPanSpeed = .5;

    shooterTiltMotorTicksPerRotation = 1024;   //Analog potentiometer 1024 units per rotation.
    tiltFangsUpperLimit = 730; //Measured Value
    tiltFangsLowerLimit = -5; //Measured Value

    //For Encoders: 10 FT = 149083 Encoder units
    //1 FT = 14908 Units
    //1 Inch = 1242 Units
    encoderTicksPerInch = 1242;
    robotLength = 35;
    robotWidth = 23;

    System.out.println("I AM FALCONBOT! CACAW! CACAAAAAWWWWW!");
  }
}

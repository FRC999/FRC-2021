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
  // Autonomous constants
  //public static int robotLength = 18;
  //public static int robotWidth = 33;
  //public static int encoderTicksPerInch = 326;

  public static boolean enablePneumatics = true;

  /**
   * Starting positions along the field. SI units. The X is a distance from the
   * edge along the long end of the field, and Y is a distance from the edge along
   * the short end distance 5.0, 13.5 in 2020/2021 games mean 5 meters along the
   * long side and in the center along the short end
   */
  public static double startingPoseX = 0.9144;
  // Note that the Slalom path requires a different value
  public static double startingPoseY = 2.286;

  // Drivetrain Motor Controllers
  public final static int frontLeftDriveMotorControllerID = 3;
  public final static int backLeftDriveMotorControllerID = 4;
  public final static int frontRightDriveMotorControllerID = 1;
  public final static int backRightDriveMotorControllerID = 2;

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
  //public final static int shooterPanMotorEncoderTicksPerRotation = 4096;
  public final static double shooterPanMotorEncoderTicksPerDegree = 11.22;
  //public  static int shooterPanMotorEncoderTicksPerRotation = 178;
  public final static int shooterXResolution = 640;
  public final static int shooterYResolution = 240; 
  public final static int shooterResolutionAcceptableError = 10;
  public final static double shooterPanSpeed = -.1;
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
  public static int tiltFangs10Feet = 700;


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
  public static boolean isSplitStick;

  // The difference between the left and right side encoder values when the robot
  // is rotated 180 degrees
  // Allowable error to exit movement methods
  public static int defaultAcceptableError = 250;
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

  /* Must be ported to new DriveSubsystem archetecture for use
  // Closed loop Aux PID parameter values
  public final static double P_1 = 2.0 * fullMotorOutput / encoderUnitsPerShaftRotation; 
  // 75% motor output when error = one rotation
  public final static double I_1 = 0.005 * fullMotorOutput / encoderUnitsPerShaftRotation;
  public final static double D_1 = 0.1;
  public final static double F_1 = 2.8; // just a guesstimate
  public final static int Izone_1 = 500;
  public final static double PeakOutput_1 = 1;
*/

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
  public final static int tiltDefaultAcceptableError = 2;
  
  // MotionMagic curve smoothing parameter [0 - 8]
  public final static int tiltSmoothing =3;

  public final static int tiltCruiseVelocity = 50;
  public final static int tiltAcceleration = 50;


  // Allowable error to exit vision tracking movement methods
  public final static int allowableLeft = ((RobotMap.shooterXResolution / 2) - (RobotMap.shooterResolutionAcceptableError));
  public final static int allowableRight = ((RobotMap.shooterXResolution / 2) + (RobotMap.shooterResolutionAcceptableError));
  public final static int allowableBelow = ((RobotMap.shooterYResolution / 2) - (RobotMap.shooterResolutionAcceptableError));
  public final static int allowableAbove = ((RobotMap.shooterYResolution / 2) + (RobotMap.shooterResolutionAcceptableError));

  public final static double encoderTicksPerDegreeX = 11;  // for Turret Encoder
  public final static double potentiometerTicksPerDegreeY = 1; // TODO check this value
  public final static double pixelsPerDegreeX = 12;  //based on lifecam having a 53.4 degree viewing angle and 640 horizontal pixels
  public final static double pixelsPerDegreeY = 15.2; //480 vertical pixels / 31.6 degree viewing angle

  /**
   * Talon PID methods often demand slot ID's, so we need to do this :(
   */
  public final static int SLOT_0 = 0;

  public static final boolean enableShooter = false;
}

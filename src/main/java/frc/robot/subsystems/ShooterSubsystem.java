package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ShooterStandbyCommand;
import frc.robot.commands.ShooterVisionCommand;

public class ShooterSubsystem extends Subsystem {

  public static WPI_VictorSPX shooterMotorController = new WPI_VictorSPX(RobotMap.shooterWheelMotorControllerID);
  public static WPI_TalonSRX panMotorController = new WPI_TalonSRX(RobotMap.shooterPanMotorControllerID);
  public WPI_TalonSRX tiltMotorController = new WPI_TalonSRX(RobotMap.ShooterTiltMotorControllerID);
  public Boolean fangsActivated = false;
  // double shooterSpeed = 0.5;

  NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  public void configureShooterControllers() {
    shooterMotorController.configFactoryDefault();
    panMotorController.configFactoryDefault();

    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    panMotorController.configFeedbackNotContinuous(true, RobotMap.configureTimeoutMs);
    

    tiltMotorController.configFactoryDefault();
    tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    tiltMotorController.setSelectedSensorPosition(0);
    /*
     * Johnson motors have a quadrature encoder with 178 ticks per revolution. Notes
     * on wiring them to talons: type |motor | breakout board --------------------
     * 5v |brown | red output 1/A |yellow| green output 2/B |green | yellow ground
     * |blue | black
     * 
     */

  }
  public void configurePanMotorControllerForPosition() {
    // Configure the encoders for PID control
    panMotorController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, RobotMap.PID_PAN,
        RobotMap.configureTimeoutMs);

    /* Configure motor neutral deadband */
    panMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    panMotorController.configPeakOutputForward(+1, RobotMap.configureTimeoutMs);
    panMotorController.configPeakOutputReverse(-1, RobotMap.configureTimeoutMs);
    panMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
    panMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

    /* FPID Gains for pan motor */

    panMotorController.config_kP(RobotMap.SLOT_0, RobotMap.P_PAN, RobotMap.configureTimeoutMs);
    panMotorController.config_kI(RobotMap.SLOT_0, RobotMap.I_PAN, RobotMap.configureTimeoutMs);
    panMotorController.config_kD(RobotMap.SLOT_0, RobotMap.D_PAN, RobotMap.configureTimeoutMs);
    panMotorController.config_kF(RobotMap.SLOT_0, RobotMap.F_PAN, RobotMap.configureTimeoutMs);
    panMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
    panMotorController.configAllowableClosedloopError(RobotMap.SLOT_0, RobotMap.panDefaultAcceptableError, RobotMap.configureTimeoutMs);

    panMotorController.configMotionAcceleration(RobotMap.panAcceleration, RobotMap.configureTimeoutMs);
    panMotorController.configMotionCruiseVelocity(RobotMap.panCruiseVelocity, RobotMap.configureTimeoutMs);
    panMotorController.configMotionSCurveStrength(RobotMap.panSmoothing);
    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    panMotorController.configClosedLoopPeriod(1, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);

  } // End configurePanMotorControllerForPosition

  public void configureTiltMotorControllerForPosition() {

    tiltMotorController.setSensorPhase(true);
    tiltMotorController.setInverted(true);
      // Configure the encoders for PID control
    tiltMotorController.configSelectedFeedbackSensor(FeedbackDevice.Analog, RobotMap.PID_TILT,	RobotMap.configureTimeoutMs);

    /* Configure motor neutral deadband */
    tiltMotorController.configNeutralDeadband(RobotMap.NeutralDeadband, RobotMap.configureTimeoutMs);

    /**
    * Max out the peak output (for all modes).  
    * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
    */
    tiltMotorController.configPeakOutputForward(+0.5, RobotMap.configureTimeoutMs);
    tiltMotorController.configPeakOutputReverse(-0.5, RobotMap.configureTimeoutMs);
    tiltMotorController.configNominalOutputForward(0, RobotMap.configureTimeoutMs);
    tiltMotorController.configNominalOutputReverse(0, RobotMap.configureTimeoutMs);

    /* FPID Gains for each side of drivetrain */
    tiltMotorController.config_kP(RobotMap.SLOT_0, RobotMap.P_TILT, RobotMap.configureTimeoutMs);
    tiltMotorController.config_kI(RobotMap.SLOT_0, RobotMap.I_TILT, RobotMap.configureTimeoutMs);
    tiltMotorController.config_kD(RobotMap.SLOT_0, RobotMap.D_TILT, RobotMap.configureTimeoutMs);
    tiltMotorController.config_kF(RobotMap.SLOT_0, RobotMap.F_TILT, RobotMap.configureTimeoutMs);
    tiltMotorController.configClosedLoopPeakOutput(RobotMap.SLOT_0, RobotMap.PeakOutput_0, RobotMap.configureTimeoutMs);
    tiltMotorController.configAllowableClosedloopError(RobotMap.SLOT_0, RobotMap.tiltDefaultAcceptableError, RobotMap.configureTimeoutMs);

    tiltMotorController.configMotionAcceleration(RobotMap.tiltAcceleration, RobotMap.configureTimeoutMs);
    tiltMotorController.configMotionCruiseVelocity(RobotMap.tiltCruiseVelocity, RobotMap.configureTimeoutMs);
    tiltMotorController.configMotionSCurveStrength(RobotMap.tiltSmoothing);

    /**
    * 1ms per loop.  PID loop can be slowed down if need be.
    * For example,
    * - if sensor updates are too slow
    * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
    * - sensor movement is very slow causing the derivative error to be near zero.
    */
    tiltMotorController.configClosedLoopPeriod(0, RobotMap.closedLoopPeriodMs, RobotMap.configureTimeoutMs);
  } // End configureTiltMotorControllerForPosition

  public int getPanEncoder() {
    return panMotorController.getSelectedSensorPosition();
  }

  // public int getTiltEncoder() {
  // return tiltMotorController.getSelectedSensorPosition();
  // }

  /** stops the shooter motor. */
  public void standby() {
    shooterMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void shoot(double shooterSpeed) {
    shooterMotorController.set(ControlMode.PercentOutput, -shooterSpeed);  
  }

  public  double deadbandPan(double pan) {
    if (Math.abs(pan) >= RobotMap.deadbandZ){
      if (pan > 0) {
        pan = (pan - RobotMap.deadbandZ) /(1-RobotMap.deadbandZ); 
      }
      else{
        pan = (pan + RobotMap.deadbandZ) /(1-RobotMap.deadbandZ);  
      }
    }
    else {
      pan = 0;
    } 
    return pan;
  }

  public void pan(double pan) {
    panMotorController.set(ControlMode.PercentOutput, deadbandPan(pan));
  }

  public void panStandby() {
    panMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void panToSetpoint(int setpoint){
    panMotorController.set(ControlMode.Position, setpoint);
  }

  /**
   * gets the x-value of the center of the object the camera is looking at. 640 is
   * the maximum; if it returns 1000, the pi is not posting to networktables.
   */
  public double getX() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("CenterXCoord").getDouble(1000);// 640 is the maximum;
  }

  /**
   * gets the y-value of the center of the object the camera is looking at. 480 is
   * the maximum; if it returns 1000, the pi is not posting to networktables.
   */
  public double getY() {
    return networkTableInstance.getTable("TestTable/PI").getEntry("CenterYCoord").getDouble(1000);// 480 is the maximum;
  }

  /**
   * tests whether the current x-value of the object the robot sees is in the
   * center of its field of view. Outputs the difference (current value - 320).
   */
  public double differenceFromMiddleX() {
    return Math.abs((getX() - (RobotMap.shooterXResolution / 2)));
  }

  /**
   * tests whether the current y-value of the object the robot sees is in the
   * center of its field of view. Outputs the difference (current value - 240).
   */
  public double differenceFromMiddleY() {
    return (getY() - (RobotMap.shooterYResolution / 2));
  }
  public double getPanError() {
    return panMotorController.getClosedLoopError();// Returns the PID error for Pan motion control;
  }


  public String whichSide() {
    String state = "";
    if (getX() != 1000) {
      state = "Center";
      if (getX() <= (RobotMap.allowableLeft)) { // Find Value 
        state = "Left";
      } else if (getX() >= (RobotMap.allowableRight)) { // Find value 
        state = "Right";
      } else if (getX() > RobotMap.allowableLeft && getX() < RobotMap.allowableRight) {
        state = "Center";
      }
    }
    return state;
  }

  public String whichVerticalSide(){
    String state = "";
    if (getY() != 1000) {
      state = "Center";
      if (getY() <= (RobotMap.allowableBelow)) { // 310
        state = "Below";
      } else if (getY() >= (RobotMap.allowableAbove)) { // 330
        state = "Above";
      } else if (getY() > RobotMap.allowableBelow && getY() < RobotMap.allowableAbove) {
        state = "Center";
      }
    }
    else state = "Park";
    return state; 
   }

   public void centerShooterTilt() {
     if (getY() < 1000) {
       tiltMotorController.set(ControlMode.Position, Math.round(729 - getY()));  
      }
    } 

  /*
  public void centerShooterTilt() {
    switch (whichVerticalSide()) {
      case "Below": {
        tiltMotorController.set(ControlMode.Position, Math.round(
            getTiltPot() + (differenceFromMiddleY() / RobotMap.pixelsPerDegreeY * RobotMap.potentiometerTicksPerDegreeY)));
      }
        break;

      case "Center": {
        tiltStandby();
        //tiltMotorController.set(ControlMode.PercentOutput, 0);
      }
        break;

      case "Above": {
        tiltMotorController.set(ControlMode.Position, Math.round(
            getTiltPot() - (differenceFromMiddleY() / RobotMap.pixelsPerDegreeY * RobotMap.potentiometerTicksPerDegreeY)));
      }
        break;

        case "Park": {
          tiltMotorController.set(ControlMode.Position, 200);
        }
          break;
  
      default: {
        tiltStandby();
        //tiltMotorController.set(ControlMode.PercentOutput, 0);
      }
    }
  }
*/

  public void centerShooterPan() {
    switch (whichSide()) {
      case "Left": {

        panMotorController.set(ControlMode.Position, Math.round(
            getPanEncoder() - (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
      }
        break;

      case "Center": {
        panStandby();
        //panMotorController.set(ControlMode.PercentOutput, 0);
      }
        break;

      case "Right": {
        panMotorController.set(ControlMode.Position, Math.round(
            getPanEncoder() + (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
      }
        break;

      default: {
        panStandby();
        //panMotorController.set(ControlMode.PercentOutput, 0);
      }
    }
  }
  public void centerShooterPan(String side, double lastError) {
    if (Math.abs(lastError) < Math.abs(getPanError())+1) {
      panMotorController.set(ControlMode.Position, lastError, DemandType.Neutral, 0);
    }
    switch (side) {
      case "Left": {
        panMotorController.set(ControlMode.Position, Math.round(
            getPanEncoder() - (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
      }
        break;

      case "Center": {
        panStandby();
        //panMotorController.set(ControlMode.PercentOutput, 0);
      }
        break;

      case "Right": {
        panMotorController.set(ControlMode.Position, Math.round(
            getPanEncoder() + (differenceFromMiddleX() / RobotMap.pixelsPerDegreeX * RobotMap.encoderTicksPerDegreeX)));
      }
        break;

      default: {
        panStandby();
        //panMotorController.set(ControlMode.PercentOutput, 0);
      }
    }
  }

//Start of work on the FANGS

public void zeroTiltPot() {
  tiltMotorController.setSelectedSensorPosition(0);
}

  public int getTiltPot() {
    return tiltMotorController.getSelectedSensorPosition();
    //1024 units per rotation
  }

  public void tiltStandby() {
    tiltMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void tiltGoToSetpoint(int setpoint) {
    tiltMotorController.set(ControlMode.Position, setpoint);
  }

  public void tiltFangDeployToggle(){
    if (fangsActivated==false)
    {
    tiltMotorController.set(ControlMode.Position, RobotMap.tiltFangsUpperLimit);
    fangsActivated = true;
    }

    else if (fangsActivated==true)
    {
      tiltMotorController.set(ControlMode.Position, RobotMap.tiltFangsLowerLimit);
      fangsActivated = false;
    }
  } 

  public void testPanTurret(){
    //System.out.println("Testing PAN");
    double maxSpeed = 0.25; 
    double output = (Robot.oi.turnStick.getZ()*1) * maxSpeed;
    panMotorController.set(ControlMode.PercentOutput, output);
  }

  public void testTiltFangs(){
    //System.out.println("Testing TILT");
    double maxSpeed = 0; //sets motorspeed to 0 if tilt is against stops
    //if (getTiltPot() >= RobotMap.tiltFangsLowerLimit &&  getTiltPot() <= RobotMap.tiltFangsUpperLimit) {
      maxSpeed = 0.25;
    //}
    double output = (Robot.oi.driveStick.getThrottle()*-1) * maxSpeed;
    Robot.smartDashboardSubsystem.updateShooterValues();
    tiltMotorController.set(ControlMode.PercentOutput, output);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ShooterStandbyCommand());
    //setDefaultCommand(new ShooterVisionCommand());
  }

}
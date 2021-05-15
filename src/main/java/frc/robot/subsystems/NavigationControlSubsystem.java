// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.controller.RamseteController;

/** Add your docs here. */
public class NavigationControlSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DriveSubsystemBase driveSubsystem;
  private NavXSubsystem navX;
  private DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;
  private RamseteController ramseteController = new RamseteController();

  private SimpleMotorFeedforward feedforward;

  private PIDController leftPidController;
  private PIDController rightPidController;

  public NavigationControlSubsystem(DriveSubsystemBase driveSubsystem, NavXSubsystem navXSubsystem) {
    this.driveSubsystem = driveSubsystem; // Instance variable shadowed by local variable
    navX = navXSubsystem;
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(driveSubsystem.distanceBetweenWheels));
    leftPidController = new PIDController(driveSubsystem.trajectoryRioPidP_Value,
      driveSubsystem.trajectoryRioPidI_Value0, driveSubsystem.trajectoryRioPidD_Value0);
    rightPidController = new PIDController(driveSubsystem.trajectoryRioPidP_Value,
      driveSubsystem.trajectoryRioPidI_Value0, driveSubsystem.trajectoryRioPidD_Value0);

    feedforward = new SimpleMotorFeedforward(driveSubsystem.feedForwardStatic, 
      driveSubsystem.feedForwardVelocity, driveSubsystem.feedForwardAcceleration);

    /**
     * Note that DifferentialDriveOdometry contructor was revised since team 5190
     * posted their video The parameters listed here were gathered from WPI
     * documentation as well as the document created by Team 8027. 
     * 
     * The initial heading of the navX need not be zeroed for this to function.
     */
    odometry = new DifferentialDriveOdometry(navX.getHeading(),
        new Pose2d(RobotMap.startingPoseX, RobotMap.startingPoseY, new Rotation2d()));
  }
  /**
   * Return robot pose to starting position (as set in RobotMap)
   * Note that we face due east by default, and that there is not currently a lever
   * to change that.
   */
  public void resetPose(){
    resetPose(new Pose2d(RobotMap.startingPoseX, RobotMap.startingPoseY, 
      new Rotation2d()));
  }

  /**
   * This method sets the pose to the robot.
   * @param startingPose is the new position of the robot.
   */
  public void resetPose(Pose2d startingPose){
    /**
     * This method of odometry assumes that encoders are zeroed.  That fact is 
     * documented, but your author missed it in his rush, and wasted precious time
     * trying to debug it.
     */
    odometry.resetPosition(startingPose, navX.getHeading());
    driveSubsystem.zeroDriveEncoders();
  }
  public void updateOdometer() {
    Rotation2d gyroAngle = navX.getHeading();

    double leftDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getLeftEncoder());
    double rightDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getRightEncoder());
    Robot.smartDashboardSubsystem.updateMeterPrint(leftDistanceMeters, rightDistanceMeters);

    odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters);
  }

  public double convertEncoderTicsToMeters(int encoderTics){
    return Units.inchesToMeters(encoderTics / driveSubsystem.getEncoderTicksPerInch());
  }

  public double convertMetersToEncoderTics(double meters){
    return Units.metersToInches(meters)*driveSubsystem.getEncoderTicksPerInch();
  }

    /**
   * Gets the current Pose2d of the robot
   */
  public Pose2d getPosition(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    double leftSpeed = convertEncoderTicsToMeters(driveSubsystem.getLeftEncoderSpeed());
    double rightSpeed = convertEncoderTicsToMeters(driveSubsystem.getRightEncoderSpeed());
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  /**
   * Use for RIO-based pid, sets the left- and right- motor voltages
   * @param left Voltage for left motor, output from WPI PidController
   * @param right Voltage for right motor, output from WPI PidController
   */
  public void setMotorVoltages(double left, double right){
    driveSubsystem.setLeftVoltage(left);
    driveSubsystem.setRightVoltage(right);
  }

  /**
   * This attempts to drive the wheels to reach the given velocities
   * @param leftSpeedMeters speed of left side in meters per second
   * @param rightSpeedMeters speed of right side in meters per second
   */
  public void setMotorSpeeds(double leftSpeedMeters, double rightSpeedMeters){
    /**
     * While encoder positions must be ints, velocities can be doubles
     * Let's use doubles for that bit of extra precision
     */
    double leftSpeedTics, rightSpeedTics; 
    leftSpeedTics = convertMetersToEncoderTics(leftSpeedMeters);
    rightSpeedTics = convertMetersToEncoderTics(rightSpeedMeters);

    //Speeds need to be in tics per 100ms
    leftSpeedTics /= 10;
    rightSpeedTics /= 10;

    driveSubsystem.velocityPid(leftSpeedTics, rightSpeedTics);
  }

  public static Trajectory getTrajectory(String trajectoryName){
    String trajectoryFile = "output/" + trajectoryName + ".json";
    Trajectory trajectory = new Trajectory();
    Path trajectoryPath;
    System.out.println(Filesystem.getDeployDirectory().toPath());
    System.out.println(trajectoryFile);
    trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile);
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
      System.out.println(trajectoryPath);
    }
    return trajectory;
  }
  
  public RamseteController getRamseteController() {
    return ramseteController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public PIDController getRightPidController() {
    return rightPidController;
  }

  public PIDController getLeftPidController() {
    return leftPidController;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  @Override
  protected void initDefaultCommand() {
    // Required method, does nothing
  }
}

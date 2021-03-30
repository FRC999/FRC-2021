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
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.controller.RamseteController;

/** Add your docs here. */
public class NavigationControlSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DriveSubsystemBase driveSubsystem;
  private NavXSubsystem navX;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private RamseteController ramseteController = new RamseteController();

  // Values set to null to prevent accidental explosions / death
  // (which would be bad)
  private SimpleMotorFeedforward feedforward = null;// new SimpleMotorFeedforward(0, 0, 0);

  private PIDController leftPidController;
  private PIDController rightPidController;

  public NavigationControlSubsystem(DriveSubsystemBase driveSubsystem, NavXSubsystem navXSubsystem) {
    this.driveSubsystem = driveSubsystem; // Instance variable shadowed by local variable
    navX = navXSubsystem;
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotMap.distanceBetweenWheels));
    leftPidController = new PIDController(driveSubsystem.talonPidP_Value0,
      driveSubsystem.talonPidI_Value0, driveSubsystem.talonPidD_Value0);
    rightPidController = new PIDController(driveSubsystem.talonPidP_Value0,
      driveSubsystem.talonPidI_Value0, driveSubsystem.talonPidD_Value0);

    /**
     * Note that DifferentialDriveOdometry contructor was revised since team 5190
     * posted their video The parameters listed here were gathered from WPI
     * documentation as well as the document created by Team 8027. I also assume
     * that the initial vector was zeroed properly in the Robot class.
     */
    odometry = new DifferentialDriveOdometry(navX.getHeading(),
        new Pose2d(RobotMap.startingPoseX, RobotMap.startingPoseY, new Rotation2d()));
  }

  public void updateOdometer() {
    Rotation2d gyroAngle = navX.getHeading();
    double leftDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getLeftEncoder());
    double rightDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getRightEncoder());
    odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters);
  }

  public double convertEncoderTicsToMeters(int encoderTics){
    return Units.inchesToMeters(encoderTics / RobotMap.encoderTicksPerInch);
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

  public static Trajectory getTrajectory(String trajectoryName){
    trajectoryName += ".json";
    Trajectory trajectory = new Trajectory();
    Path trajectoryPath;
    System.out.println(Filesystem.getDeployDirectory().toPath());
    trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
    System.out.println("ol mcodonald");
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
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

public void setMotorSpeeds(Double left, Double right) {
}
}

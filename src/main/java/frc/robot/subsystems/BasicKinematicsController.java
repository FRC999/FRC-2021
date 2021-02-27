// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.controller.RamseteController;

/** Add your docs here. */
public class BasicKinematicsController extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DriveSubsystemBase driveSubsystem;
  private NavXSubsystem navX;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private RamseteController ramseteController = new RamseteController();

  // Values set to null to prevent accidental explosions / death
  //                                    (which would be bad)
  private SimpleMotorFeedforward feedforward = null;// new SimpleMotorFeedforward(0, 0, 0);

  private PIDController leftPidController = null; //new PIDController(0, 0, 0);
  private PIDController rightPidController = null;

  public BasicKinematicsController(DriveSubsystemBase driveSubsystem, NavXSubsystem navXSubsystem){
    this.driveSubsystem = driveSubsystem; //Instance variable shadowed by local variable
    navX = navXSubsystem;
    kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(RobotMap.distanceBetweenWheels));

    /** Note that DifferentialDriveOdometry contructor was revised since team 5190 posted their video
     * The parameters listed here were gathered from WPI documentation as well as the document
     * created by Team 8027. I also assume that the initial vector was zeroed properly in the Robot class.
     */
    odometry = new DifferentialDriveOdometry(navX.getHeading(),
      new Pose2d(RobotMap.startingPoseX, RobotMap.startingPoseY, new Rotation2d()));
  }

  /**
   * Gets the current Pose2d of the robot
   */
  public Pose2d getPosition(){
    return odometry.getPoseMeters();
  }
  
  public void updateOdometer(){
    driveSubsystem.getLeftEncoder();
    Rotation2d gyroAngle = navX.getHeading();
    double leftDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getLeftEncoder());
    double rightDistanceMeters = convertEncoderTicsToMeters(driveSubsystem.getRightEncoder());
    odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters);
  }

  public double convertEncoderTicsToMeters(int encoderTics){
    return Units.inchesToMeters(encoderTics / RobotMap.encoderTicksPerInch);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

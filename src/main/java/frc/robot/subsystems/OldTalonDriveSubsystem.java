/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
/**
 * Add your docs here. TODO: Add docs
 */
public class OldTalonDriveSubsystem extends DriveSubsystemBase {
  // Put methods for controlling this subsystem here. Call these from Commands.
  static final int frontLeftTalonID = -1;
  static final int frontRightTalonID = -1;
  static final int backLeftTalonID = -1;
  static final int backRightTalonID = -1;
  static final int midLeftTalonID = -1;
  static final int midRightTalonID = -1;

  static WPI_TalonSRX frontLeftDriveTalonSRX = new WPI_TalonSRX(frontLeftTalonID);
  static WPI_TalonSRX backLeftDriveTalonSRX = new WPI_TalonSRX(backLeftTalonID);
  static WPI_TalonSRX midLeftDriveTalonSRX = new WPI_TalonSRX(midLeftTalonID);
  static WPI_TalonSRX frontRightDriveTalonSRX = new WPI_TalonSRX(frontRightTalonID);
  static WPI_TalonSRX backRightDriveTalonSRX = new WPI_TalonSRX(backRightTalonID);
  static WPI_TalonSRX midRightDriveTalonSRX = new WPI_TalonSRX(midRightTalonID);

  

  public OldTalonDriveSubsystem(){
	  frontLeftDriveMotorController = frontLeftDriveTalonSRX;
	  backLeftDriveMotorController = backLeftDriveTalonSRX;
	  frontRightDriveMotorController = frontRightDriveTalonSRX;
    backRightDriveMotorController = backRightDriveTalonSRX;
    drive = new DifferentialDrive(frontLeftDriveTalonSRX, frontRightDriveTalonSRX);
  }

  public void resetDriveTrainControllers(){
    super.resetDriveTrainControllers();

    midLeftDriveTalonSRX.configFactoryDefault();
    midRightDriveTalonSRX.configFactoryDefault();

    midLeftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    midRightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    
    midLeftDriveTalonSRX.follow(frontLeftDriveMotorController);
    midRightDriveTalonSRX.follow(frontRightDriveMotorController);

    midLeftDriveTalonSRX.setInverted(InvertType.FollowMaster);
    midRightDriveTalonSRX.setInverted(InvertType.FollowMaster);
  }

  public void configureEncoders(){
    frontLeftDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    frontRightDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

}

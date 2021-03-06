/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here. TODO: Add docs
 */
public class DriveSubsystemFrankenbot extends DriveSubsystemBase {
  // Put methods for controlling this subsystem here. Call these from Commands.

  static WPI_TalonSRX frontLeftDriveTalonSRX = new WPI_TalonSRX(RobotMap.frontLeftDriveMotorControllerID);
  static WPI_TalonSRX backLeftDriveTalonSRX = new WPI_TalonSRX(RobotMap.backLeftDriveMotorControllerID);
  static WPI_TalonSRX frontRightDriveTalonSRX = new WPI_TalonSRX(RobotMap.frontRightDriveMotorControllerID);
  static WPI_TalonSRX backRightDriveTalonSRX = new WPI_TalonSRX(RobotMap.backRightDriveMotorControllerID);

  public DriveSubsystemFrankenbot(){
	  frontLeftDriveMotorController = frontLeftDriveTalonSRX;
	  backLeftDriveMotorController = backLeftDriveTalonSRX;
	  frontRightDriveMotorController = frontRightDriveTalonSRX;
    backRightDriveMotorController = backRightDriveTalonSRX;
    drive = new DifferentialDrive(frontLeftDriveTalonSRX, frontRightDriveTalonSRX);

    talonPidP_Value0 = 0.75 * RobotMap.fullMotorOutput / RobotMap.encoderUnitsPerShaftRotation;
    talonPidI_Value0 = 0.005 * RobotMap.fullMotorOutput / RobotMap.encoderUnitsPerShaftRotation;
    talonPidD_Value0 = .1;
    talonPidF_Value0 = 0.227; // TODO: Investigate more to see if we actually intend to use static FF's


    talonPidCruiseVelocity =2250;
    talonPidAcceleration =2250;
    talonPidSmoothing = 3;
  }

  public void configureEncoders() {
    frontLeftDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    frontRightDriveMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  @Override
  public void setLeftVoltage(double voltage) {
    frontLeftDriveTalonSRX.setVoltage(voltage);
  }

  @Override
  public void setRightVoltage(double voltage) {
    frontRightDriveTalonSRX.setVoltage(voltage);
  }
}

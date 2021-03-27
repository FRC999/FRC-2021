/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * Please note that PID constants are set in the constructor
 */
public class DriveSubsystemFalconBot extends DriveSubsystemBase {

  static WPI_TalonFX frontLeftDriveTalonFX = new WPI_TalonFX(RobotMap.frontLeftDriveMotorControllerID);
  static WPI_TalonFX backLeftDriveTalonFX = new WPI_TalonFX(RobotMap.backLeftDriveMotorControllerID);
  static WPI_TalonFX frontRightDriveTalonFX = new WPI_TalonFX(RobotMap.frontRightDriveMotorControllerID);
  static WPI_TalonFX backRightDriveTalonFX = new WPI_TalonFX(RobotMap.backRightDriveMotorControllerID);

  public DriveSubsystemFalconBot(){
	  frontLeftDriveMotorController = frontLeftDriveTalonFX;
	  backLeftDriveMotorController = backLeftDriveTalonFX;
	  frontRightDriveMotorController = frontRightDriveTalonFX;
    backRightDriveMotorController = backRightDriveTalonFX;
    drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);

    encoderUnitsPerShaftRotation = 2048;
    encoderUnitsPerRobotRotation = 3925;// thats the SUM of the two (this is just a rough guess)
    // distanceBetweenWheels = ????; //TODO: Measure
    robotLength = 35;
    robotWidth = 23;


    talonPidP_Value0 = 2 * RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    talonPidI_Value0 = 0.005 * RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    talonPidD_Value0 = .1;
    talonPidF_Value0 = 2.8; // TODO: Investigate more to see if we actually intend to use static FF's

    talonPidCruiseVelocity = 2250 * 6;
    talonPidAcceleration = 2250 *2;
    talonPidSmoothing = 3;
  }

  @Override
  public void configureEncoders() {
    //empty, because the default for the Falcon is to use the integrated controller
  }

  @Override
  public void setLeftVoltage(double voltage) {
    frontLeftDriveTalonFX.setVoltage(voltage);
  }

  @Override
  public void setRightVoltage(double voltage) {
    frontRightDriveTalonFX.setVoltage(voltage);
  }

}

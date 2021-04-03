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
    encoderUnitsPerRobotRotation = 80700;// thats the SUM of the two (this is just a rough guess
    distanceBetweenWheels = 20;
    wheelDiameter = 6;
    robotLength = 35;
    robotWidth = 23;

    // I am pretty sure that the configuration tool doesn't want the gains modified --JW
    talonPidP_Value0 = 0.141;// * RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    talonPidI_Value0 = 0.00;// * RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    talonPidD_Value0 = 0;
    
    feedForwardStatic = 0.566;
    feedForwardVelocity = 0.0272;
    feedForwardAcceleration = 0.00308;

    talonPidCruiseVelocity = 2250*5;
    talonPidAcceleration = 2250 *2;
    talonPidSmoothing = 3;
  
    //Gains for MotionMagic
    MMtalonPidP_Value0 = 0.2 ;//* RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    MMtalonPidI_Value0 = 0.005 ;//* RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    MMtalonPidD_Value0 = 0.1;
    MMtalonPidF_Value0 = 2.8;
    
    MMtalonPidCruiseVelocity = 2250*2;
    MMtalonPidAcceleration = 2250 *2;
    MMtalonPidSmoothing = 3;
  
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

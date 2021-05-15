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
    encoderGearReduction = 11.25;
    encoderUnitsPerRobotRotation = 66500;// thats the SUM of the two (this is just a rough guess
    distanceBetweenWheels = 20;
    wheelDiameter = 6;
    robotLength = 35;
    robotWidth = 23;
    gearboxReduction = 11.25; // enter 1 if there is no reduction

    // I am pretty sure that the configuration tool doesn't want the gains modified --JW
    trajectoryRioPidP_Value = 1.43;// * RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    trajectoryRioPidI_Value0 = 0.00;// * RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    trajectoryRioPidD_Value0 = 0;
    
    feedForwardStatic = 0.541;
    feedForwardVelocity = 0.305;
    feedForwardAcceleration = 0.0362;

    motionMagicCruiseVelocity = 2250*5;
    motionMagicAcceleration = 2250 *2;
    motionMagicSmoothing = 3;
  
    //Gains for MotionMagic
    motionMagicPidP_Value = 0.75 ;//* RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    motionMagicPidI_Value = 0.005 ;//* RobotMap.fullMotorOutput / encoderUnitsPerShaftRotation;
    motionMagicPidD_Value = 0.01;
    motionMagicPidF_Value = 2;
    
    motionMagicCruiseVelocity = 2250*3;
    motionMagicAcceleration = 2250 *3;
    motionMagicSmoothing = 3;
  
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

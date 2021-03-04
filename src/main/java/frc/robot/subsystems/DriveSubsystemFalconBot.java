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
    RobotMap.IAmFalconBot();
	  frontLeftDriveMotorController = frontLeftDriveTalonFX;
	  backLeftDriveMotorController = backLeftDriveTalonFX;
	  frontRightDriveMotorController = frontRightDriveTalonFX;
    backRightDriveMotorController = backRightDriveTalonFX;
    drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);


    // TODO: Determine if these values pulled from RobotMap are for falcon or talon bot
    talonPidP_Value0 = 0.75 * RobotMap.fullMotorOutput / RobotMap.encoderUnitsPerShaftRotation;
    talonPidI_Value0 = 0.005 * RobotMap.fullMotorOutput / RobotMap.encoderUnitsPerShaftRotation;
    talonPidD_Value0 = .1;
    talonPidF_Value0 = 0.227; // TODO: Investigate more to see if we actually intend to use static FF's

    // Values are definitely guesses
    talonPidCruiseVelocity =2250;
    talonPidAcceleration =2250;
    talonPidSmoothing = 3;
  }
 

  //public static DifferentialDrive drive = new DifferentialDrive(frontLeftDriveTalonFX, frontRightDriveTalonFX);
  // No differential or arcade drive for falcons

  @Override
  public void configureEncoders() {
    //empty, because the default for the Falcon is to use the integrated controller
  }
}

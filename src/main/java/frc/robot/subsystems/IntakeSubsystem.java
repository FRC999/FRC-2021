/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeStandbyCommand;


public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_VictorSPX magazineRightMotorController = new WPI_VictorSPX(RobotMap.magazineRightMotorControllerID);
  static WPI_TalonSRX magazineLeftMotorController = new WPI_TalonSRX(RobotMap.magazineLeftMotorControllerID);
  static WPI_VictorSPX intakeMotorController = new WPI_VictorSPX(RobotMap.intakeMotorControllerID);
  static WPI_VictorSPX loaderMotor1Controller = new WPI_VictorSPX(RobotMap.loaderMotor1ControllerID);
  static WPI_VictorSPX loaderMotor2Controller = new WPI_VictorSPX(RobotMap.loaderMotor2ControllerID);

  public static DoubleSolenoid intakeSolenoid;

  public IntakeSubsystem(){
    if(RobotMap.enablePneumatics){
      intakeSolenoid = new DoubleSolenoid(RobotMap.IntakeSolenoidForwardChannel,RobotMap.IntakeSolenoidReverseChannel);
    }
  }

  public void standby(){
    magazineLeftMotorController.set(ControlMode.PercentOutput, 0);
    magazineRightMotorController.set(ControlMode.PercentOutput, 0);
    intakeMotorController.set(ControlMode.PercentOutput, 0);
    loaderMotor1Controller.set(ControlMode.PercentOutput, 0);
    loaderMotor2Controller.set(ControlMode.PercentOutput, 0);
  }

  public void intake(double motorSpeed){
    intakeMotorController.set(ControlMode.PercentOutput, motorSpeed);
  }

  public void magazine(double motorSpeed){
    magazineLeftMotorController.set(ControlMode.PercentOutput, -motorSpeed);
    magazineRightMotorController.set(ControlMode.PercentOutput, -motorSpeed);
  }

  public void loader(double motorSpeed){
    loaderMotor1Controller.set(ControlMode.PercentOutput, -motorSpeed);
    loaderMotor2Controller.set(ControlMode.PercentOutput, -motorSpeed);
  }

  public void IntakeUp() {
    if(RobotMap.enablePneumatics){
      intakeSolenoid.set(Value.kForward);
    }
  }

  public void IntakeDown() {
    if(RobotMap.enablePneumatics){
      intakeSolenoid.set(Value.kReverse);
    }
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeStandbyCommand());
  }
}

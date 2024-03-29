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

  static WPI_VictorSPX intakeMotorController = new WPI_VictorSPX(RobotMap.intakeMotorControllerID);
  static WPI_TalonSRX loaderFrontMotorController = new WPI_TalonSRX(RobotMap.loaderFrontMotorControllerID);
  static WPI_VictorSPX loaderRearMotorController = new WPI_VictorSPX(RobotMap.loaderRearMotorControllerID);

  public static DoubleSolenoid intakeSolenoid;

  public IntakeSubsystem(){
    if(RobotMap.enablePneumatics){
      intakeSolenoid = new DoubleSolenoid(RobotMap.IntakeSolenoidForwardChannel,RobotMap.IntakeSolenoidReverseChannel);
    }
  }

  public void standby(){
    intakeMotorController.set(ControlMode.PercentOutput, 0);
    loaderFrontMotorController.set(ControlMode.PercentOutput, 0);
    loaderRearMotorController.set(ControlMode.PercentOutput, 0);
  }

  public void intake(double motorSpeed){
    intakeMotorController.set(ControlMode.PercentOutput, motorSpeed);
  }

  public void loader(double motorSpeed){
    loaderFrontMotorController.set(ControlMode.PercentOutput, -motorSpeed);
    loaderRearMotorController.set(ControlMode.PercentOutput, -motorSpeed);
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

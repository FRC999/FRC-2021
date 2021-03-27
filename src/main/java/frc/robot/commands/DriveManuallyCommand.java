/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveManuallyCommand extends Command {
   static double turnAdjust = 0.6;

  public DriveManuallyCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
    setInterruptible(true);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Man. Drive Setup");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Read joystick values
    double move = (Robot.oi.driveStick.getY() * -1); // inverts sign for Y axis
    double turn = 0;
    if (RobotMap.isSplitStick) {
      turn = Robot.oi.turnStick.getX();
    }
    else{
      turn = Robot.oi.driveStick.getX();
     }
    //double turn = Robot.oi.driveStick.getX();
    Robot.driveSubsystem.manualDrive(move, turn * turnAdjust);
    Robot.smartDashboardSubsystem.updateEncoderValue();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }


  

}

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

public class AimFangsManuallyCommand extends Command {
  public AimFangsManuallyCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shooterSubsystem);
    
  }
 /* public double panVal() {
    double pan = 0;
      if (Robot.oi.driveStick.getZ() > 0) { // Panning right (clockwise)
      if (Robot.shooterSubsystem.getPanEncoder() >= 1406) {
    pan = 0;
    return pan;
      } else {
        pan = 0;
        return pan;
      }
    } else { //Panning left (counterclockwise)
      if (Robot.shooterSubsystem.getPanEncoder() > 435) {
        if (Robot.shooterSubsystem.getPanEncoder() <= 3340) {
         pan = 0;
        return pan;
        }
      } else {
        pan = Robot.oi.driveStick.getZ();
        return pan;
      }
    }
    return pan;
  }
  */
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Robot.shooterSubsystem.fangsActivated != true) {
      Robot.shooterSubsystem.tiltFangDeployToggle();
    }
    //set shooter wheel to full speed
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("PAN : " + Robot.shooterSubsystem.getPanEncoder());
    System.out.println("TILT : " + Robot.shooterSubsystem.getTiltPot());
    Robot.shooterSubsystem.manualAimTiltFangs();
    if(Robot.shooterSubsystem.getTiltPot() < RobotMap.tiltFangsLowerLimit || Robot.shooterSubsystem.getTiltPot() > RobotMap.tiltFangsUpperLimit) {
      Robot.shooterSubsystem.tiltFangDeployToggle();
    }
    System.out.println(Robot.shooterSubsystem.tiltValue);
    /*
    Robot.shooterSubsystem.shoot(-1);
    //use the twist and throttle to control shooter pan and tilt
    //double tilt = Robot.oi.driveStick.getThrottle();
    Robot.shooterSubsystem.pan(Robot.oi.driveStick.getZ());
    //Robot.shooterSubsystem.tilt(tilt);
    Robot.smartDashboardSubsystem.updateShooterValues();
    */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.shooterSubsystem.fangsActivated) {
        return false;
    } else {
        return true;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooterSubsystem.tiltStandby();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
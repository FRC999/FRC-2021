/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoSlalomPathCommandGroup extends CommandGroup {
  private static double targetAngle = 0;
  /**
   * Add your docs here.
   */
  public AutoSlalomPathCommandGroup() {
   
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
    targetAngle = -45;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(106));
    //targetAngle = (current heading - desired heading)*-1
    targetAngle = (Robot.navXSubsystem.getYaw()-(0))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(104));
    targetAngle = (Robot.navXSubsystem.getYaw()-(45))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(106));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-45))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(58));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-135))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(50));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-225))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(100));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-180))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(124));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-135))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(100));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-180))*-1;
    addSequential(new DriveTurnCommand(-targetAngle));
    addSequential(new DriveForwardCommand(24));
    addSequential(new DriveStopCommand());

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}

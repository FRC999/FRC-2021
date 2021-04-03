/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoBarrelRunCommandGroup extends CommandGroup {
  private static double targetAngle = 0;

  /**
   * Add your docs here.
   */
  public AutoBarrelRunCommandGroup() {

    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
    targetAngle = 26;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(178));
    //targetAngle = (current heading - desired heading)*-1
    targetAngle = (Robot.navXSubsystem.getYaw()-(7))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(-111));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-18))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(215));
    targetAngle = (Robot.navXSubsystem.getYaw()-(13))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(-99));
    targetAngle = (Robot.navXSubsystem.getYaw()-(47))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(155));
    targetAngle = (Robot.navXSubsystem.getYaw()-(118))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(-52));
    targetAngle = (Robot.navXSubsystem.getYaw()-(178))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(310));
    addSequential(new DriveStopCommand());

     //***Relative turning based on encoder readings ..No heading corrections */
    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
/** 
    addSequential(new DriveTurnCommand(26));
    addSequential(new DriveForwardCommand(65));
    addSequential(new DriveTurnCommand(-19));
    addSequential(new DriveForwardCommand(-111));
    addSequential(new DriveTurnCommand(-25));
    addSequential(new DriveForwardCommand(215));
    addSequential(new DriveTurnCommand(31));
    addSequential(new DriveForwardCommand(-99));
    addSequential(new DriveTurnCommand(34));
    addSequential(new DriveForwardCommand(155));
    addSequential(new DriveTurnCommand(71));
    addSequential(new DriveForwardCommand(-52));
    addSequential(new DriveTurnCommand(60));
    addSequential(new DriveForwardCommand(310));
    addSequential(new DriveStopCommand());
*/
   



    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

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

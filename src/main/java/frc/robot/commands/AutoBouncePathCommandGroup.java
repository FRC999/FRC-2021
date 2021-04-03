/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoBouncePathCommandGroup extends CommandGroup {
  private static double targetAngle = 0;

  /**
   * Add your docs here.
   */
  public AutoBouncePathCommandGroup() {

    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
    targetAngle = -62;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(65));
    //targetAngle = (current heading - desired heading)*-1
    targetAngle = (Robot.navXSubsystem.getYaw()-(-118))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(-106));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-76))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(95));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-103))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(-92));
    targetAngle = (Robot.navXSubsystem.getYaw()-(0))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(52));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-90))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(92));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-123))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(-65));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-180))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(-12));
    addSequential(new DriveStopCommand());

     //***Relative turning based on encoder readings ..No heading corrections */
    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
/** 
    addSequential(new DriveTurnCommand(-153));
    addSequential(new DriveForwardCommand(65));
    addSequential(new DriveTurnCommand(-55));
    addSequential(new DriveForwardCommand(-106));
    addSequential(new DriveTurnCommand(42));
    addSequential(new DriveForwardCommand(95));
    addSequential(new DriveTurnCommand(-27));
    addSequential(new DriveForwardCommand(-92));
    addSequential(new DriveTurnCommand(84));
    addSequential(new DriveForwardCommand(52));
    addSequential(new DriveTurnCommand(-90));
    addSequential(new DriveForwardCommand(92));
    addSequential(new DriveTurnCommand(-35));
    addSequential(new DriveForwardCommand(-65));
    addSequential(new DriveTurnCommand(-55));
    addSequential(new DriveForwardCommand(-12));
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

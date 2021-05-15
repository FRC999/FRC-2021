/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoGalacticSearchCommandGroup extends CommandGroup {
  private static double targetAngle = 0;

  /**
   * Add your docs here.
   */
  public AutoGalacticSearchCommandGroup() {

    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
    addSequential(new IntakeDownCommand());
    addSequential(new IntakeInCommand());
    targetAngle = 90;
    addSequential(new DriveForwardCommand(60));
    //targetAngle = (current heading - desired heading)*-1
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(30));
    targetAngle = (Robot.navXSubsystem.getYaw()-(25))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(64));
    targetAngle = (Robot.navXSubsystem.getYaw()-(44))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(41));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-90))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(32));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-62))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(67));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-90))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(30));
    targetAngle = (Robot.navXSubsystem.getYaw()-(45))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(191));
    addSequential(new DriveStopCommand());

     //***Relative turning based on encoder readings ..No heading corrections */
    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
/** 
    addSequential(new IntakeDownCommand());
    addSequential(new IntakeInCommand());
    addSequential(new DriveForwardCommand(60));
    addSequential(new DriveTurnCommand(90));
    addSequential(new DriveForwardCommand(30));
    addSequential(new DriveTurnCommand(-65));
    addSequential(new DriveForwardCommand(64));
    addSequential(new DriveTurnCommand(19));
    addSequential(new DriveForwardCommand(-41));
    addSequential(new DriveTurnCommand(-136));
    addSequential(new DriveForwardCommand(32));
    addSequential(new DriveTurnCommand(28));
    addSequential(new DriveForwardCommand(67));
    addSequential(new DriveTurnCommand(-28));
    addSequential(new DriveForwardCommand(30));
    addSequential(new DriveTurnCommand(135));
    addSequential(new DriveForwardCommand(191));
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

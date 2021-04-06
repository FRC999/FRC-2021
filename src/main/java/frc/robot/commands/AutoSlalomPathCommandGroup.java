/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;

public class AutoSlalomPathCommandGroup extends CommandGroup {
  private static double targetAngle = 0;
  /**
   * Add your docs here.
   */
  public AutoSlalomPathCommandGroup() {
   /** 
    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg
    targetAngle = -45;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(106));
    //targetAngle = (current heading - desired heading)*-1
    addSequential(new WaitCommand(.1));
    targetAngle = (Robot.navXSubsystem.getYaw()-(0))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(104));
    addSequential(new WaitCommand(.1));
    targetAngle = (Robot.navXSubsystem.getYaw()-(45))*-1;
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(106));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-45))*-1;
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(58));
    addSequential(new WaitCommand(.1));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-135))*-1;
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(50));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-225))*-1;
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(100));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-180))*-1;
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new DriveForwardCommand(124));
    addSequential(new WaitCommand(.1));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-135))*-1;
    addSequential(new DriveTurnCommand(targetAngle));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(100));
    targetAngle = (Robot.navXSubsystem.getYaw()-(-180))*-1;
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-targetAngle));
    addSequential(new DriveForwardCommand(24));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveStopCommand());


 addSequential(new DriveForwardCommand(6));
    addSequential(new DriveTurnCommand(-45));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(106));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(47));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(111));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(45));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(96));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-90));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(58));
    addSequential(new WaitCommand(.1));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-90));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(18));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-45));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(36));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-90));
    addSequential(new DriveForwardCommand(48));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(90));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(124));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(45));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(100));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-45));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(24));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveStopCommand());
 

    */    
     //***Relative turning based on encoder readings ..No heading corrections */
    //DriveForwardCommand arguments are in inches
    //DriveTurnCommand wants arguments in relative angle cw = pos, ccw = neg

/** 
    Warning The following code was written when the DriveForwardCommand used encoderunits

    addSequential(new DriveForwardCommand(40500));
    addSequential(new DriveTurnCommand(-64));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(82000));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(65));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(188000));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(90));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(53400));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-91));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(59000));
    addSequential(new WaitCommand(.1));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-93));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(43300));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-95));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(43300));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-95));
    addSequential(new DriveForwardCommand(57400));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(90));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(208000));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(61));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(51500));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveTurnCommand(-40));
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand(100));
    addSequential(new WaitCommand(.1));
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

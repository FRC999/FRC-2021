/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonomousTrajectoryRioCommand;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.MoveOffLineAuto;
import frc.robot.commands.RealSmartAutoCommand;
import frc.robot.commands.ShootWithAcesCommand;
import frc.robot.commands.ShooterVisionCommand;
import frc.robot.subsystems.NavigationControlSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystemBase;
import frc.robot.subsystems.DriveSubsystemFrankenbot;
import frc.robot.subsystems.DriveSubsystemFalconBot;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  NetworkTable table;
  public static DriveSubsystemBase driveSubsystem;
  public static NavigationControlSubsystem navigationSubsystem;
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static NavXSubsystem navXSubsystem = new NavXSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // public static ControlPanelSubsystem controlPanelSubsystem = new
  // ControlPanelSubsystem();
  public static ShuffleboardSubsystem shuffleBoardSubsystem = new ShuffleboardSubsystem();
  // public static Command visionCommand = new ShooterVisionCommand();


  public boolean TestBool = false;
  public static OI oi;
  Command autonomousCommand;

  // Sendable choosers belowP
  SendableChooser<Command> sendableCommandChooser = new SendableChooser<Command>();
  SendableChooser<String> sendableStringChooser = new SendableChooser<String>();
  SendableChooser<Integer> sendableIntegerChooser = new SendableChooser<Integer>();
  SendableChooser<Double> sendableDoubleChooser = new SendableChooser<Double>();
  // End sendable choosers

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Set up shuffleboard
    shuffleBoardSubsystem.setupShuffleboard();

    NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    ntInst.startClientTeam(999);
    NetworkTable table = ntInst.getTable("TestTable");
    NetworkTableEntry testEntry = table.getEntry("test");
    testEntry.setDouble(10.5);
    System.out.println("Hit robotInit");

    /**
     * Zero Z axis; used in DriveSubsystembase to determine initial heading in
     * Kinematics driving so, do it before running the contructor for that class
     */
    Robot.navXSubsystem.zeroYaw();

    RobotMap.isFalconBot = true;
    System.out.println("falconBotSwitch = "+ RobotMap.isFalconBot);

    // Change to reflect current robot
    driveSubsystem = new DriveSubsystemFrankenbot();
    System.out.println("Type of drive subsystem: " + driveSubsystem.getClass());
    RobotMap.isSplitStick = true;
    
    driveSubsystem.setDefaultCommand(new DriveManuallyCommand());

    sendableCommandChooser.addOption("Really Smart Auto", new RealSmartAutoCommand());
    sendableCommandChooser.addOption("Move Off Line", new MoveOffLineAuto());

    SmartDashboard.putData("auto chooser", sendableCommandChooser);
    Robot.driveSubsystem.resetDriveTrainControllers();

    // after testing run only the second configure method
    Robot.driveSubsystem.configureDriveTrainControllersForSimpleMagic();

    Robot.driveSubsystem.zeroDriveEncoders();
    Robot.driveSubsystem.driveTrainBrakeMode();

    // Don't start kinematics untill we're ready
    navigationSubsystem = new NavigationControlSubsystem(driveSubsystem, navXSubsystem);


    Robot.shooterSubsystem.configureShooterControllers();
    Robot.shooterSubsystem.configurePanMotorControllerForPosition();
    Robot.shooterSubsystem.configureTiltMotorControllerForPosition();
    Robot.shooterSubsystem.zeroTiltPot();
    

    oi = new OI();
    sendableCommandChooser.setDefaultOption("Hello Alan!", new AutonomousTrajectoryRioCommand("TestTrajectory"));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // shooterSubsystem.getPanEncoder();
    smartDashboardSubsystem.updateAllDisplays();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    driveSubsystem.driveTrainCoastMode();
    // visionCommand.cancel();
    // controlPanelSubsystem.stopTalon();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    autonomousCommand = sendableCommandChooser.getSelected();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */
    driveSubsystem.driveTrainBrakeMode();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    } else
      System.out.println("Auto is null.");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

    driveSubsystem.driveTrainBrakeMode();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    smartDashboardSubsystem.updateNavXValues();
    smartDashboardSubsystem.updateEncoderValue();

    // controlPanelSubsystem.putSeenColor();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * OPERATOR INPUT This class is the glue that binds the controls on the physical
 * operator interface to the commands and command groups that allow control of
 * the robot.
 */
public class OI {
 
  public static int driveStickPort = 0;
  public static int copilotPort = 2;
  public static int turnStickPort = 1;

  public Joystick driveStick = new Joystick(driveStickPort);
  public Joystick turnStick = new Joystick(turnStickPort);
  public Joystick buttonBox = new Joystick(copilotPort);

  //driveStick
  Button IntakeInButton = new JoystickButton(driveStick, 1);
  Button IntakeOutButton = new JoystickButton(driveStick, 2);
  Button IntakeDownButton = new JoystickButton(driveStick, 3);
  Button IntakeUpButton = new JoystickButton(driveStick, 4);
  Button visionTracking = new JoystickButton(driveStick, 5);
  Button zeroTurret = new JoystickButton(driveStick, 6);
  Button TiltManual = new JoystickButton(driveStick,7);
  //Button runTrajectory = new JoystickButton(driveStick,11)   ;
  //int turret = new turnStick.getPov();
  Button ZeroFangsButton = new JoystickButton(driveStick, 8); 
  Button ShooterStayOnButton = new JoystickButton(driveStick, 10); 
  //Button DriveForwardButton = new JoystickButton(driveStick, 11); 
 // Button DriveBackwardButton = new JoystickButton(driveStick, 12); 
  //Button StatusReport = new JoystickButton(driveStick,11);
  

  //turnsStick
  Button ShooterWheelOnButton = new JoystickButton(turnStick, 1);
  Button turretButton = new JoystickButton(turnStick, 2);
  Button LoaderUpButton = new JoystickButton(turnStick, 3);
  Button LoaderDownButton = new JoystickButton(turnStick, 4);
  Button ShooterTiltSetpointButton = new JoystickButton(turnStick, 5);
  Button ShooterTiltZeroButton = new JoystickButton(turnStick, 6);
  Button ShooterWheelManual = new JoystickButton(turnStick, 7);
  Button ShooterNavLockButton = new JoystickButton(turnStick, 8);
  Button AutoTest1Button = new JoystickButton(turnStick, 9);
  //Button AutoTest2Button = new JoystickButton(turnStick, 12);
  Button ZeroYawButton = new JoystickButton(turnStick, 10);
 

  //Button Board
// *** Temporary reasign buttons for testing
  Button GotoGreenZoneButton = new JoystickButton(buttonBox, 1);
  Button ShootingFromGreenZoneButton = new JoystickButton(buttonBox, 2);
  Button GotoYellowZoneButton = new JoystickButton(buttonBox, 4);
  Button GotoBlueZoneButton = new JoystickButton(buttonBox, 7);
  Button GotoRedZoneButton = new JoystickButton(buttonBox, 10);
  Button GotoEndZoneButton = new JoystickButton(buttonBox, 12);
 // Button ZeroYawButton = new JoystickButton(turnStick, 10);
 // Button LoaderUpButton = new JoystickButton(buttonBox, 7);
 // Button LoaderDownButton = new JoystickButton(buttonBox, 8);
  //Button LoaderStandbyButton = new JoystickButton(buttonBox, 9);
  //Button ShooterWheelOnButton = new JoystickButton(buttonBox, 10);
  //Button LoadAndShootButton = new JoystickButton(buttonBox, 11);
  //Button ShooterStandbyButton = new JoystickButton(buttonBox, 12);
  
// *** End Temporary Code
  
  //Button visionTilt = new JoystickButton(buttonBox, 2);
  //Button magazineInward = new JoystickButton(buttonBox, 4);
  //Button loaderUp = new JoystickButton(buttonBox, 5);
  //Button shooterMotor = new JoystickButton(buttonBox, 6);
  //Button magazineOutward = new JoystickButton(buttonBox, 7);
  //Button loaderDown = new JoystickButton(buttonBox, 8);
  //Button fangsFullyBack = new JoystickButton(buttonBox, 9);
  //Button visionTracking = new JoystickButton(buttonBox, 10);
  //Button fullShooter = new JoystickButton(buttonBox, 11); //loader + magazine + shooter
  //Button zeroTurret = new JoystickButton(buttonBox, 12);
  public OI() {
     // Setup All Commands Here
     
     //right Joystick
     //runIntake.whileHeld(new IntakeInCommand());
     //runIntake.whenReleased(new IntakeUpCommand());
     //intakeEject.whileHeld(new IntakeEject());
     ShooterTiltSetpointButton.whileHeld(new ShooterTiltGoToSetpointCommand(706));
     ShooterTiltZeroButton.whileHeld(new ShooterTiltGoToZeroCommand());
     //runTrajectory.whenPressed(new AutonomousTrajectoryRioCommand("TestTrajectory"));
     //AutoSlalomButton.whenPressed(new AutoBouncePathCommandGroup());
     //AutoSlalomButton.whenPressed(new AutoSlalomPathCommandGroup());
     //AutoTest1Button.whenPressed(new DriveTurnCommand(-90));
     //AutoTest2Button.whenPressed(new DriveTurnCommand(90));

     AutoTest1Button.whenPressed(new DriveZeroEncodersCommand());
     //AutoTest2Button.whenPressed(new DriveForwardCommand(-60));
     ZeroYawButton.whenPressed( new NavXZeroYawCommand());
     GotoGreenZoneButton.whenPressed( new GoToShootingPositionCommand(-54,706));
     GotoYellowZoneButton.whenPressed( new GoToShootingPositionCommand(-110,706));
     GotoBlueZoneButton.whenPressed( new GoToShootingPositionCommand(-155,710));
     GotoRedZoneButton.whenPressed( new GoToShootingPositionCommand(-215,690));
     GotoEndZoneButton.whenPressed( new GoToShootingPositionCommand(-215,706));
     ShootingFromGreenZoneButton.whenPressed( new ShooterRunWheelCommand(0.5));
     ShooterNavLockButton.whenPressed(new ShooterLockTurretToHeadingCommand(0));
     //Left Joystick



    //Button Board
// *** Temporary reasign buttons to Fang control
    //climberSolenoidForward.whenPressed(new ClimbExtendCommand());
    //winchMotor.whileHeld(new ClimbWinchUpCommand());
    //climberSolenoidReverse.whenPressed(new ClimbRetractCommand());

    //ShooterTiltUp10.whileHeld(new ShooterTiltUp10Command());
    //ShooterTiltToZero.whileHeld(new ShooterTiltGoToZeroCommand());
    //ShooterTiltDown10.whileHeld(new ShooterTiltDown10Command());
    
    //magazineInward.whileHeld(new IntakeMagazineInCommand());
    LoaderUpButton.whileHeld(new IntakeLoaderUpCommand());
    LoaderDownButton.whileHeld(new IntakeLoaderDownCommand());
    //LoaderStandbyButton.whileHeld(new IntakeStandbyCommand());
    ShooterWheelOnButton.whileHeld(new ShooterRunWheelCommand(1));
    //ShooterWheelOnButton.whenReleased(new ShooterStandbyCommand());
    ShooterStayOnButton.whenPressed(new ShooterRunWheelCommand(1));
    IntakeDownButton.whileHeld(new IntakeDownCommand());
    IntakeUpButton.whileHeld(new IntakeUpCommand());
    IntakeInButton.whileHeld(new IntakeInCommand());
    IntakeOutButton.whileHeld(new IntakeReverseCommand());
    //IntakeStandbyButton.whileHeld(new IntakeStandbyCommand());
    //LoadAndShootButton.whileHeld(new ShooterFullCommand());
    //ShooterStandbyButton.whileHeld(new ShooterAndLoaderStopCommand());
    ZeroFangsButton.whileHeld(new ShooterZeroTiltCommand());



    //fangsFullyBack.whenPressed(new ShooterTiltGoToSetpointCommand());
    visionTracking.whenPressed(new ShooterVisionCommand());
    //DriveForwardButton.whenPressed(new DriveForwardCommand(24));
    //[DriveBackwardButton.whenPressed(new DriveForwardCommand(-24));
    //visionTilt.whileHeld(new ShooterVisionTiltCommand());
    //fullShooter.whileHeld(new ShooterFullCommand());
    turretButton.whileHeld(new ShooterPanManuallyCommand());
    zeroTurret.whileHeld(new ShooterTurretCenterCommand());
    TiltManual.whileHeld(new ShooterTiltManuallyCommand());
    ShooterWheelManual.whileHeld(new ShooterWheelSpeedManualCommand());
    //StatusReport.whileHeld(new StatusReport());

  }
}
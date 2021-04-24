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
  // TODO: This class needs SERIOUS cleanup ASAP
 
  public static int driveStickPort = 0;
  public static int turnStickPort = 1;
  public static int copilotPort = 2;

  public Joystick driveStick = new Joystick(driveStickPort);
  public Joystick turnStick = new Joystick(turnStickPort);
  public Joystick buttonBox = new Joystick(copilotPort);

  // *** driveStick
  Button DriveStickButton1 = new JoystickButton(driveStick, 1);
  Button DriveStickButton2 = new JoystickButton(driveStick, 2);
  Button DriveStickButton3 = new JoystickButton(driveStick, 3);
  Button DriveStickButton4 = new JoystickButton(driveStick, 4);
  Button DriveStickButton5 = new JoystickButton(driveStick, 5);
  Button DriveStickButton6 = new JoystickButton(driveStick, 6);
  Button DriveStickButton7 = new JoystickButton(driveStick, 7);
  Button DriveStickButton8 = new JoystickButton(driveStick, 8);
  Button DriveStickButton9 = new JoystickButton(driveStick, 9);
  Button DriveStickButton10 = new JoystickButton(driveStick, 10);
  Button DriveStickButton11 = new JoystickButton(driveStick, 11);
  Button DriveStickButton12 = new JoystickButton(driveStick, 12);

  // *** turnStick
  Button TurnStickButton1 = new JoystickButton(turnStick, 1);
  Button TurnStickButton2 = new JoystickButton(turnStick, 2);
  Button TurnStickButton3 = new JoystickButton(turnStick, 3);
  Button TurnStickButton4 = new JoystickButton(turnStick, 4);
  Button TurnStickButton5 = new JoystickButton(turnStick, 5);
  Button TurnStickButton6 = new JoystickButton(turnStick, 6);
  Button TurnStickButton7 = new JoystickButton(turnStick, 7);
  Button TurnStickButton8 = new JoystickButton(turnStick, 8);
  Button TurnStickButton9 = new JoystickButton(turnStick, 9);
  Button TurnStickButton10 = new JoystickButton(turnStick, 10);
  Button TurnStickButton11 = new JoystickButton(turnStick, 11);
  Button TurnStickButton12 = new JoystickButton(turnStick, 12);

  // *** buttonBox
  Button ButtonBoxButton1 = new JoystickButton(buttonBox, 1);
  Button ButtonBoxButton2 = new JoystickButton(buttonBox, 2);
  Button ButtonBoxButton3= new JoystickButton(buttonBox, 3);
  Button ButtonBoxButton4 = new JoystickButton(buttonBox, 4);
  Button ButtonBoxButton5 = new JoystickButton(buttonBox, 5);
  Button ButtonBoxButton6 = new JoystickButton(buttonBox, 6);
  Button ButtonBoxButton7 = new JoystickButton(buttonBox, 7);
  Button ButtonBoxButton8 = new JoystickButton(buttonBox, 8);
  Button ButtonBoxButton9 = new JoystickButton(buttonBox, 9);
  Button ButtonBoxButton10 = new JoystickButton(buttonBox, 10);
  Button ButtonBoxButton11 = new JoystickButton(buttonBox, 11);
  Button ButtonBoxButton12 = new JoystickButton(buttonBox, 12);

  public OI() {

    // *** DriveStick Commands Here
    DriveStickButton1.whileHeld(new IntakeInCommand());
    DriveStickButton2.whileHeld(new IntakeReverseCommand());
    DriveStickButton3.whileHeld(new IntakeDownCommand());
    DriveStickButton4.whileHeld(new IntakeUpCommand());
    DriveStickButton5.whenPressed(new ShooterVisionCommand());
    DriveStickButton6.whileHeld(new ShooterTurretCenterCommand());
    DriveStickButton7.whileHeld(new ShooterTiltManuallyCommand());
    DriveStickButton8.whileHeld(new ShooterZeroTiltCommand());
    //DriveStickButton9.whenPressed(new Command());
    DriveStickButton10.whenPressed(new ShooterRunWheelCommand(1));
    //DriveStickButton11.whenPressed(new Command());
    DriveStickButton12.whenPressed(new AutonomousTrajectoryRioCommand("Slalom.wpilib"));
    DriveStickButton12.whenReleased(new DriveStopCommand());

    // *** TurnStick Commands Here
    TurnStickButton1.whileHeld(new ShooterRunWheelCommand(1));
    TurnStickButton2.whileHeld(new ShooterPanManuallyCommand());
    TurnStickButton3.whileHeld(new IntakeLoaderUpCommand());
    TurnStickButton4.whileHeld(new IntakeLoaderDownCommand());
    TurnStickButton5.whileHeld(new ShooterTiltGoToSetpointCommand(706));
    TurnStickButton6.whileHeld(new ShooterTiltGoToZeroCommand());
    TurnStickButton7.whileHeld(new ShooterRunWheelCommand(1));
    TurnStickButton8.whenPressed(new ShooterLockTurretToHeadingCommand(0));
    TurnStickButton9.whenPressed(new DriveZeroEncodersCommand());
    TurnStickButton10.whenPressed( new NavXZeroYawCommand());
    //TurnStickButton11.whenPressed( new Command());
    TurnStickButton12.whileHeld(new ShooterWheelSpeedManualCommand());

    // *** ButtonBox Commands Here
    ButtonBoxButton1.whenPressed( new GoToShootingPositionCommand(-54,706));//GotoGreenZoneButton
    ButtonBoxButton2.whenPressed( new ShooterRunWheelCommand(0.5));//ShootingFromGreenZoneButton
    //ButtonBoxButton3.whenPressed( new Command());
    ButtonBoxButton4.whenPressed( new GoToShootingPositionCommand(-110,706));//GotoYellowZoneButton
    //ButtonBoxButton5.whenPressed( new Command());
    //ButtonBoxButton6.whenPressed( new Command());
    ButtonBoxButton7.whenPressed( new GoToShootingPositionCommand(-155,710));//GotoBlueZoneButton
    //ButtonBoxButton8.whenPressed( new Command());
    //ButtonBoxButton9.whenPressed( new Command());
    ButtonBoxButton10.whenPressed( new GoToShootingPositionCommand(-215,690));//GotoRedZoneButton
    //ButtonBoxButton11.whenPressed( new Command());
    ButtonBoxButton12.whenPressed( new GoToShootingPositionCommand(-215,706));//GotoEndZoneButton
 
    // TODO:  Decide how we want to assign buttons when not split stick configuration.

  }
}
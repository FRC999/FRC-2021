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
  public static int copilotPort = 1;
  public static int turnStickPort = 2;

  public Joystick driveStick = new Joystick(driveStickPort);
  public Joystick turnStick = new Joystick(turnStickPort);
  public Joystick buttonBox = new Joystick(copilotPort);

  //driveStick
  Button runIntake = new JoystickButton(driveStick, 1);
  Button turretButton = new JoystickButton(driveStick, 2);
  Button intakeEject = new JoystickButton(driveStick, 3);
  Button visionTilt = new JoystickButton(driveStick, 5);
  Button ShooterTiltZeroButton = new JoystickButton(driveStick, 6); 
  Button TiltManual = new JoystickButton(driveStick,7);
  Button runTrajectory = new JoystickButton(driveStick,10);
  Button StatusReport = new JoystickButton(driveStick,11);
  //int turret = new turnStick.getPov();
  //turnsStick

  //Button Board
// *** Temporary reasign buttons to Fang control
  //Button climberSolenoidForward = new JoystickButton(buttonBox, 1);
  //Button winchMotor = new JoystickButton(buttonBox, 2);
  //Button climberSolenoidReverse = new JoystickButton(buttonBox, 3);
  Button ShooterTiltUp10 = new JoystickButton(buttonBox, 1);
  Button ShooterTiltToZero = new JoystickButton(buttonBox, 2);
  Button ShooterTiltDown10 = new JoystickButton(buttonBox, 3);
// *** End Temporary Code
  
  //Button visionTilt = new JoystickButton(buttonBox, 2);
  Button magazineInward = new JoystickButton(buttonBox, 4);
  Button loaderUp = new JoystickButton(buttonBox, 5);
  Button shooterMotor = new JoystickButton(buttonBox, 6);
  Button magazineOutward = new JoystickButton(buttonBox, 7);
  Button loaderDown = new JoystickButton(buttonBox, 8);
  Button fangsFullyBack = new JoystickButton(buttonBox, 9);
  Button visionTracking = new JoystickButton(buttonBox, 10);
  Button fullShooter = new JoystickButton(buttonBox, 11); //loader + magazine + shooter
  Button zeroTurret = new JoystickButton(buttonBox, 12);
  public OI() {
     // Setup All Commands Here
     
     //right Joystick
     runIntake.whileHeld(new IntakeInCommand());
     runIntake.whenReleased(new IntakeUpCommand());
     intakeEject.whileHeld(new IntakeEject());
     //ShooterTiltSetpointButton.whileHeld(new ShooterTiltGoToSetpointCommand());
     ShooterTiltZeroButton.whileHeld(new ShooterTiltGoToZeroCommand());
     runTrajectory.whenPressed(new AutonomousTrajectoryRioCommand("TestTrajectory"));
     //Left Joystick



    //Button Board
// *** Temporary reasign buttons to Fang control
    //climberSolenoidForward.whenPressed(new ClimbExtendCommand());
    //winchMotor.whileHeld(new ClimbWinchUpCommand());
    //climberSolenoidReverse.whenPressed(new ClimbRetractCommand());

    ShooterTiltUp10.whileHeld(new ShooterTiltUp10Command());
    ShooterTiltToZero.whileHeld(new ShooterTiltGoToZeroCommand());
    ShooterTiltDown10.whileHeld(new ShooterTiltDown10Command());
    
    magazineInward.whileHeld(new IntakeMagazineInCommand());
    loaderUp.whileHeld(new IntakeLoaderUpCommand());
    shooterMotor.whileHeld(new ShooterRunWheelCommand());
    magazineOutward.whileHeld(new IntakeMagazineInCommand());
    loaderDown.whileHeld(new IntakeLoaderDownCommand());
    fangsFullyBack.whenPressed(new ShooterTiltGoToSetpointCommand());
    visionTracking.whenPressed(new ShooterVisionCommand());
    visionTilt.whileHeld(new ShooterVisionTiltCommand());
    fullShooter.whileHeld(new ShooterFullCommand());
    turretButton.whileHeld(new ShooterPanManuallyCommand());
    zeroTurret.whileHeld(new ShooterTurretCenterCommand());
    TiltManual.whileHeld(new ShooterTiltManuallyCommand());
    StatusReport.whileHeld(new StatusReport());
  }
}
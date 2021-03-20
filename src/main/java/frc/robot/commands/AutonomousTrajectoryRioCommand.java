// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Robot;
import frc.robot.subsystems.NavigationControlSubsystem;
 
/** This form of the class runs all code on the RoboRIO*/
public class AutonomousTrajectoryRioCommand extends RamseteCommandWpilib {

    AutonomousTrajectoryRioCommand(Trajectory trajectory){
        super(
            trajectory, 
            () -> {return Robot.navigationSubsystem.getPosition();}, // Lambda supplies pose for robot
            Robot.navigationSubsystem.getRamseteController(), // Grab kinematics controller from Robot.java
            Robot.navigationSubsystem.getFeedforward(), 
            Robot.navigationSubsystem.getKinematics(), 
            () -> {return Robot.navigationSubsystem.getWheelSpeeds();}, 
            Robot.navigationSubsystem.getLeftPidController(), 
            Robot.navigationSubsystem.getRightPidController(), 
            (Double left, Double right) -> {  // yes, I DO mean the object type.
                Robot.navigationSubsystem.setMotorVoltages(left, right);
            },
            Robot.navigationSubsystem, Robot.driveSubsystem // Set requirements
        );

    }
    public AutonomousTrajectoryRioCommand(String alpha){
        this(NavigationControlSubsystem.getTrajectory(alpha));
    }
    
    public void execute(){
        Robot.navigationSubsystem.updateOdometer();
        super.execute();
    }
}

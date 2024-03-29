// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Robot;
import frc.robot.subsystems.NavigationControlSubsystem;
 
/** This form of the class runs all code on the RoboRIO*/
public class AutonomousTrajectoryTalonCommand extends RamseteCommandWpilib {

    AutonomousTrajectoryTalonCommand(Trajectory trajectory) {
        super(
            trajectory, 
            () -> {return Robot.navigationSubsystem.getPosition();}, // Lambda supplies pose for robot
            Robot.navigationSubsystem.getRamseteController(), // Grab kinematics controller from Robot.java
            Robot.navigationSubsystem.getKinematics(), 
            (Double left, Double right) -> { // yes, I DO mean the object type.
                Robot.driveSubsystem.velocityPid(left, right);
            }, 
            Robot.navigationSubsystem, Robot.driveSubsystem // Set requirements
        );
    }

    public AutonomousTrajectoryTalonCommand(String alpha) {
        this(NavigationControlSubsystem.getTrajectory(alpha));
        System.out.println("initalized trajectory command");
    }
    
    public void execute(){
        Robot.navigationSubsystem.updateOdometer();
        super.execute();
    }
}

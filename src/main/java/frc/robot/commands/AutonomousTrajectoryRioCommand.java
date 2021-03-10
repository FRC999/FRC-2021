// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Robot;
import frc.robot.subsystems.NavigationControlSubsystem;

/** This form of the class runs all code on the RoboRIO*/
public class AutonomousTrajectoryRioCommand extends RamseteCommandWpilib {
    NavigationControlSubsystem navigation;

    AutonomousTrajectoryRioCommand(Trajectory trajectory, NavigationControlSubsystem navigatorSubsystem){
        super(
            trajectory, 
            () -> {return navigatorSubsystem.getPosition();}, // Lambda supplies pose for robot
            navigatorSubsystem.getRamseteController(), // Grab kinematics controller from Robot.java
            navigatorSubsystem.getFeedforward(), 
            navigatorSubsystem.getKinematics(), 
            () -> {return navigatorSubsystem.getWheelSpeeds();}, 
            navigatorSubsystem.getLeftPidController(), 
            navigatorSubsystem.getRightPidController(), 
            (Double leftVoltage, Double rightVoltage) -> {  // yes, I DO mean the object type.
                System.out.println("hi");
            }
        );
        navigation = navigatorSubsystem;

    }
    
    public void execute(){
        navigation.updateOdometer();
        super.execute();
    }
}

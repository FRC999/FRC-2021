package frc.robot.commands;

import frc.robot.RobotMap;

public class AutonomousTrajectorySlalomRioCommand extends AutonomousTrajectoryRioCommand {
    private double previouStartY;

    public AutonomousTrajectorySlalomRioCommand(){
        super("Slalom.wpilib");
    }

    @Override
    public void initialize() {
        previouStartY = RobotMap.startingPoseY;
        RobotMap.startingPoseY = 0.6096; // We start near the bottom of the field
        super.initialize();
    }

    @Override
    public void end(){
        RobotMap.startingPoseY = previouStartY;
        super.end();
    }

    @Override
    public void interrupted(){
        RobotMap.startingPoseY = previouStartY;
        super.interrupted();
    }
    
}

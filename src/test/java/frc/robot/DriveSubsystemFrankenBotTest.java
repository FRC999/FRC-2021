package frc.robot;

import org.junit.Before;

import frc.robot.subsystems.DriveSubsystemFrankenbot;

public class DriveSubsystemFrankenBotTest extends DriveSubsystemBaseTest{
    DriveSubsystemFrankenbot driveMeFrank;
    
    @Before
    public void setupDriveSubsystem(){
        driveMeFrank = new DriveSubsystemFrankenbot();
        driveMeBase = driveMeFrank;
    }
}

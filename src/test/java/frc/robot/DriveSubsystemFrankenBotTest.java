package frc.robot;

import static org.junit.Assert.assertEquals;

import org.junit.*;

import frc.robot.subsystems.DriveSubsystemFrankenbot;

public class DriveSubsystemFrankenBotTest extends DriveSubsystemBaseTest{
    DriveSubsystemFrankenbot driveMeFrank;
    
    @Before
    public void setupDriveSubsystem(){
        driveMeFrank = new DriveSubsystemFrankenbot();
        driveMeBase = driveMeFrank;
    }

    @Test
    public void testEncodersCorrect(){
        assertEquals(driveMeFrank.getEncoderTicksPerInch(),326,10);
    }
}

package frc.robot;

import org.junit.*;

import frc.robot.subsystems.DriveSubsystemFalconBot;

import static org.junit.Assert.*;

public class DriveSubsystemFalconBotTest extends DriveSubsystemBaseTest {
    DriveSubsystemFalconBot driveMeFalcon;

    @Before
    public void setupDriveSubsystem(){
        driveMeFalcon = new DriveSubsystemFalconBot();
        driveMeBase = driveMeFalcon;
    }

    @Test
    public void confirmEncoderTicsPerInch(){
        assertEquals(driveMeFalcon.getEncoderTicksPerInch(), 1222, 1);
    }
}

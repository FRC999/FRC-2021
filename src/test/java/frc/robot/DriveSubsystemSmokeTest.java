package frc.robot;

import frc.robot.subsystems.DriveSubsystemBase;
import frc.robot.subsystems.DriveSubsystemFalconBot;

import org.junit.*;
import static org.junit.Assert.*;

public class DriveSubsystemSmokeTest {

    @Test
    public void testFalconBotEncoders(){
        DriveSubsystemBase driveme = new DriveSubsystemFalconBot();
        assertEquals(1222,driveme.getEncoderTicksPerInch(), 1);
    }
}

package frc.robot;

import frc.robot.subsystems.DriveSubsystemBase;

import org.junit.*;
import static org.junit.Assert.*;

public abstract class DriveSubsystemBaseTest {
    public DriveSubsystemBase driveMeBase;

    @After
    public void closeResources(){
        driveMeBase.close();
    }

    @Test
    public void testEncodersCalculation(){
        System.out.println(driveMeBase.getEncoderTicksPerInch());
        assertNotNull(driveMeBase.getEncoderTicksPerInch());
        assertTrue("Ticks/inch is way too small",driveMeBase.getEncoderTicksPerInch()> 200);
        assertTrue("Ticks/inch is way too big", driveMeBase.getEncoderTicksPerInch() < 3000);
    }
}

package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assume.assumeFalse;

import java.io.File;
import java.io.IOException;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;
import org.junit.runners.Parameterized.Parameter;
import org.junit.runners.Parameterized.Parameters;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.subsystems.NavigationControlSubsystem;

@RunWith(Parameterized.class)
public class NavigationTrajectoriesAreValid {
    @Parameters(name="{index}: {0} ")
    public static File[] zeroTrajectories(){
        return getTrajectoryDirectory().listFiles();
    }

    public static File getTrajectoryDirectory(){
        return new File("src/main/deploy/output");
    }

    @Parameter
    public File trajectoryFile;

    @Test
    public void checkThatZeroIsStart() throws IOException{
        String trajectoryName = trajectoryFile.getName();
        assumeFalse(trajectoryName.contains("Slalom")); // Not fixable by field design
        assumeFalse(trajectoryName.contains("TestTrajectory"));//TODO: remove test trajectory
        assumeFalse(trajectoryName.contains("Down")); //TODO: Fix Down trajectory to start facing forward
        assumeFalse(trajectoryName.contains("BarrelRacing"));
        assumeFalse(trajectoryName.contains("CircleOfLife"));
        Trajectory t = TrajectoryUtil.fromPathweaverJson(trajectoryFile.toPath());
        Pose2d tstate = t.sample(0).poseMeters;
        Pose2d baseState = new Pose2d(RobotMap.startingPoseX, RobotMap.startingPoseY, new Rotation2d());
        assertEquals(baseState,tstate);
    }

    public void verifyAutoOpenerWorks(File trajectoryFile){
        String trajectoryName = trajectoryFile.getName();
        //Shave off .json at end
        trajectoryName = trajectoryName.substring(0,trajectoryName.length()-4);
        System.out.println("Testing " + trajectoryName);
        Trajectory t = NavigationControlSubsystem.getTrajectory(trajectoryName);
        assertNotNull(t);
    }
    

}

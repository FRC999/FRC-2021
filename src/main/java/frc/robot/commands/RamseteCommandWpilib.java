// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the RAMSETE controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommandWpilib extends Command {
  private  Timer timer = new Timer();
  private  boolean usePid;
  private  Trajectory trajectory;
  private  Supplier<Pose2d> pose;
  private  RamseteController ramsetController;
  private  SimpleMotorFeedforward feedForwardController;
  private  DifferentialDriveKinematics kinematics;
  private  Supplier<DifferentialDriveWheelSpeeds> currentSpeeds;
  private  PIDController leftController;
  private  PIDController rightController;
  private  BiConsumer<Double, Double> outputVoltFunction;
  private  DifferentialDriveWheelSpeeds prevSpeeds;
  private  double prevTime;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param wheelSpeeds A function that supplies the speeds of the left and right sides of the robot
   *     drive.
   * @param leftController The PIDController for the left side of the robot drive.
   * @param rightController The PIDController for the right side of the robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("PMD.ExcessiveParameterList")
  public RamseteCommandWpilib(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      Subsystem... requirements) {
    this.trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    this.pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    this.ramsetController = requireNonNullParam(controller, "controller", "RamseteCommand");
    this.feedForwardController = feedforward;
    this.kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    this.currentSpeeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "RamseteCommand");
    this.leftController = requireNonNullParam(leftController, "leftController", "RamseteCommand");
    this.rightController = requireNonNullParam(rightController, "rightController", "RamseteCommand");
    this.outputVoltFunction = requireNonNullParam(outputVolts, "outputVolts", "RamseteCommand");

    this.usePid = true;

    addRequirements(requirements);
  }

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
   * the RAMSETE controller, and will need to be converted into a usable form by the user.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param follower The RAMSETE follower used to follow the trajectory.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param outputMetersPerSecond A function that consumes the computed left and right wheel speeds.
   * @param requirements The subsystems to require.
   */
  public RamseteCommandWpilib(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController follower,
      DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond,
      Subsystem... requirements) {
    this.trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    this.pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    this.ramsetController = requireNonNullParam(follower, "follower", "RamseteCommand");
    this.kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    this.outputVoltFunction = requireNonNullParam(outputMetersPerSecond, "output", "RamseteCommand");

    this.feedForwardController = null;
    this.currentSpeeds = null;
    this.leftController = null;
    this.rightController = null;

    this.usePid = false;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    prevTime = -1;
    System.out.println(trajectory);
    var initialState = trajectory.sample(0);
    prevSpeeds =
        kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    timer.reset();
    timer.start();
    if (usePid) {
      leftController.reset();
      rightController.reset();
    }
  }

  @Override
  public void execute() {
    double curTime = timer.get();
    double changeInTime = curTime - prevTime;

    if (prevTime < 0) {
      outputVoltFunction.accept(0.0, 0.0);
      prevTime = curTime;
      return;
    }

    DifferentialDriveWheelSpeeds targetWheelSpeeds =
        kinematics.toWheelSpeeds(
            ramsetController.calculate(pose.get(), trajectory.sample(curTime)));
            //System.out.println("pose: "+ pose.get() );
   
    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (usePid) {
      double leftFeedforward =
          feedForwardController.calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / changeInTime);

      double rightFeedforward =
          feedForwardController.calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / changeInTime);

      leftOutput =
          leftFeedforward
              + leftController.calculate(currentSpeeds.get().leftMetersPerSecond, leftSpeedSetpoint);

      rightOutput =
          rightFeedforward
              + rightController.calculate(
                  currentSpeeds.get().rightMetersPerSecond, rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

 //System.out.println("right "+ rightOutput+ "left"+leftOutput );
    outputVoltFunction.accept(leftOutput, rightOutput);
    prevSpeeds = targetWheelSpeeds;
    prevTime = curTime;
  }

  @Override
  public void end() {
    timer.stop();
  }

  protected void interrupted() {
    end();
  }

  private void addRequirements(Subsystem[] subsys){
    for(Subsystem sub : subsys){
      requires(sub);
    }
  }

  

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}

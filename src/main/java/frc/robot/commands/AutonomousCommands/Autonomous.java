// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AimAuto;
import frc.robot.commands.BallShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeedMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  
  /** Creates a new Autonomous. */
  public Autonomous(LimelightSubsystem m_limelight, DrivetrainSubsystem m_driveTrain, ArmSubsystem m_armSubsystem, IntakeSubsystem m_intakeSubsystem, BallShooterSubsystem m_shooterSubsystem, FeedMotorSubsystem m_feedSystem) {
    //set Constraints
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(m_driveTrain.getFeedforward(),
      Constants.kDriveKinematics, 10);
  
    //Configure Trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
      Constants.kMaxAccelerationMetersPerSecondSquared);
      config.setKinematics(Constants.kDriveKinematics);
      config.addConstraint(autoVoltageConstraint);

    //Rest to Ball1
    PathPlannerTrajectory restToBall1Path = PathPlanner.loadPath("RestToBall1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

    RamseteCommand restToBall1 = new RamseteCommand(restToBall1Path, m_driveTrain::getPose, 
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    m_driveTrain.getFeedforward(), Constants.kDriveKinematics, m_driveTrain::getSpeeds, 
    m_driveTrain.getLeftPID(), m_driveTrain.getRightPID(), m_driveTrain::setOutput, m_driveTrain);

    //Ball1 to Shoot
    PathPlannerTrajectory ball1ToShootPath = PathPlanner.loadPath("Ball1ToShoot", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

    RamseteCommand ball1ToShoot = new RamseteCommand(ball1ToShootPath, m_driveTrain::getPose, 
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    m_driveTrain.getFeedforward(), Constants.kDriveKinematics, m_driveTrain::getSpeeds, 
    m_driveTrain.getLeftPID(), m_driveTrain.getRightPID(), m_driveTrain::setOutput, m_driveTrain);

    //Shoot To Ball2
    PathPlannerTrajectory shootToBall2Path = PathPlanner.loadPath("ShootToBall2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

    RamseteCommand shootToBall2 = new RamseteCommand(shootToBall2Path, m_driveTrain::getPose, 
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    m_driveTrain.getFeedforward(), Constants.kDriveKinematics, m_driveTrain::getSpeeds, 
    m_driveTrain.getLeftPID(), m_driveTrain.getRightPID(), m_driveTrain::setOutput, m_driveTrain);

    //Reorientate to right Direction
    Trajectory reorientatePath = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(
        new Pose2d(1.66, 1.55, new Rotation2d(Math.toRadians(-145.01))),
        new Pose2d(1.66, 1.55, new Rotation2d(Math.toRadians(36.37)))
      ),
      config
    );

    RamseteCommand reorientate = new RamseteCommand(reorientatePath, m_driveTrain::getPose, 
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    m_driveTrain.getFeedforward(), Constants.kDriveKinematics, m_driveTrain::getSpeeds, 
    m_driveTrain.getLeftPID(), m_driveTrain.getRightPID(), m_driveTrain::setOutput, m_driveTrain);

    //Ball2 To Shoot
    PathPlannerTrajectory ball2ToShootPath = PathPlanner.loadPath("Ball2ToShoot", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

    RamseteCommand ball2ToShoot = new RamseteCommand(ball2ToShootPath, m_driveTrain::getPose, 
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    m_driveTrain.getFeedforward(), Constants.kDriveKinematics, m_driveTrain::getSpeeds, 
    m_driveTrain.getLeftPID(), m_driveTrain.getRightPID(), m_driveTrain::setOutput, m_driveTrain);

    addCommands(

      //reset Odometry
      new InstantCommand(()->m_driveTrain.resetOdometry(new Pose2d(6.74, 2.59, new Rotation2d(Math.toRadians(-153.43)))), m_driveTrain),

      //Move intake down
      new InstantCommand(m_armSubsystem::armExtend, m_armSubsystem),

      //Start intake and restToBall
      new ParallelCommandGroup(
        restToBall1,
        new RunCommand(m_intakeSubsystem::forward, m_intakeSubsystem).withTimeout(3)
      ),

      //ball1ToShoot
      ball1ToShoot,

      //Shoot and Aim
      new ParallelCommandGroup(
        new AimAuto(m_limelight, m_shooterSubsystem).withTimeout(1.5),
        new BallShooterCommand(m_limelight, m_shooterSubsystem, m_feedSystem).withTimeout(2)
      ),

      //Turn on Intake, shootToBall2
      new ParallelCommandGroup(
        shootToBall2,
        new RunCommand(m_intakeSubsystem::forward, m_intakeSubsystem).withTimeout(3)
      ),

      //reorientate and shootToBall2
      reorientate,
      ball2ToShoot,

      //Aim and Shoot
      new AimAuto(m_limelight, m_shooterSubsystem).withTimeout(1.5),
      new BallShooterCommand(m_limelight, m_shooterSubsystem, m_feedSystem).withTimeout(2)

    );
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AimAuto;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.ShootAtDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeedMotorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends SequentialCommandGroup {
  /** Creates a new DriveStraightCommand. */
  public DriveStraight(DrivetrainSubsystem m_driveTrain, ArmSubsystem m_armSubsystem, LimelightSubsystem m_limelight, BallShooterSubsystem m_shooterSubsystem, FeedMotorSubsystem m_feedSystem) {

    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(m_driveTrain.getFeedforward(),
      Constants.kDriveKinematics, 10);
  
    //Configure Trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
      Constants.kMaxAccelerationMetersPerSecondSquared);
      config.setKinematics(Constants.kDriveKinematics);
      config.addConstraint(autoVoltageConstraint);

    //Drive back for taxi points
    PathPlannerTrajectory driveBackwardsPath = PathPlanner.loadPath("DriveBackwards", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

    RamseteCommand driveBackwards = new RamseteCommand(driveBackwardsPath, m_driveTrain::getPose, 
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    m_driveTrain.getFeedforward(), Constants.kDriveKinematics, m_driveTrain::getSpeeds, 
    m_driveTrain.getLeftPID(), m_driveTrain.getRightPID(), m_driveTrain::setOutput, m_driveTrain);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      //reset Odometry
      new InstantCommand(()->m_driveTrain.resetOdometry(new Pose2d(6.09, 3.93, new Rotation2d(Math.toRadians(-174.52)))), m_driveTrain),

      //Move intake down
      //new InstantCommand(m_armSubsystem::armExtend, m_armSubsystem),

      driveBackwards

      //Shoot and Aim
      //new AimAuto(m_limelight, m_shooterSubsystem).withTimeout(2),
      //new ShootAtDistance(m_shooterSubsystem, m_feedSystem).withTimeout(3.5)
    );
  }
}

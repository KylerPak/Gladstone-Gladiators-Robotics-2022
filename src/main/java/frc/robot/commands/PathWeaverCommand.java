/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PathWeaverCommand extends CommandBase{
  private final DrivetrainSubsystem m_driveTrain;
  private boolean isFinished = false;
  public PathWeaverCommand(DrivetrainSubsystem driveTrain) {
    m_driveTrain = driveTrain;
   
    addRequirements(driveTrain);
  }

  Trajectory pathtraj (){
    String trajectoryJSON = "PathWeaver/pathweaver.wpilib.json";
    Trajectory trajectory = new Trajectory();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      SmartDashboard.putBoolean("Has path been found", true);
      } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      SmartDashboard.putBoolean("Has path been found", false);
      }
   return trajectory;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set Constraints
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(m_driveTrain.getFeedforward(),
      Constants.kDriveKinematics, 10);
  
    //Configure Trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
      Constants.kMaxAccelerationMetersPerSecondSquared);
    config.setKinematics(Constants.kDriveKinematics);
    config.addConstraint(autoVoltageConstraint);
  
    RamseteCommand command = new RamseteCommand(
      pathtraj(), 
      m_driveTrain::getPose, 
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      m_driveTrain.getFeedforward(), Constants.kDriveKinematics, m_driveTrain::getSpeeds,
      m_driveTrain.getLeftPID(), m_driveTrain.getRightPID(), m_driveTrain::setOutput, m_driveTrain);

    m_driveTrain.resetOdometry(pathtraj().getInitialPose()); //Cannot find Trajectory
    command.andThen(() -> m_driveTrain.setOutput(0, 0));
    isFinished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
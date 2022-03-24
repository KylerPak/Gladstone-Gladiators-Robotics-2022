/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.FeedMotorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class BallShooterCommand extends CommandBase {
  private final BallShooterSubsystem m_ballSubsystem;
  private final FeedMotorSubsystem m_feedSubsystem;
  private final LimelightSubsystem m_limelight;

  private int feedSystemTimer;
  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BallShooterCommand(LimelightSubsystem limelight, BallShooterSubsystem ballSystem, FeedMotorSubsystem feedSystem) {
    m_ballSubsystem = ballSystem;
    m_feedSubsystem = feedSystem;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, ballSystem, feedSystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedSystemTimer = 0;
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballSubsystem.shoot(m_limelight.getDistance() * 0.00511 - 0.01711); //calculated from linear regression
    feedSystemTimer++;

    if(feedSystemTimer > 50){
      m_feedSubsystem.feedBall();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballSubsystem.shootStop();
    m_feedSubsystem.stop();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
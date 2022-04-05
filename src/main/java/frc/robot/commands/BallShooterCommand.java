/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.FeedMotorSubsystem;
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class BallShooterCommand extends CommandBase {
  private final BallShooterSubsystem m_ballSubsystem;
  private final FeedMotorSubsystem m_feedSubsystem;
  private boolean feedBall = false;

  private int feedSystemTimer;
  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BallShooterCommand(BallShooterSubsystem ballSystem, FeedMotorSubsystem feedSystem) {
    m_ballSubsystem = ballSystem;
    m_feedSubsystem = feedSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballSystem, feedSystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedSystemTimer = 0;
    feedBall = false;
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballSubsystem.shootAtDistance();

    if((m_ballSubsystem.atTargetVelocity() || feedSystemTimer > 50) && feedBall == false){
      m_feedSubsystem.feedBall();
      feedBall = true;
    } else feedSystemTimer++;
    feedSystemTimer++;
    SmartDashboard.putNumber("Shoot Speed", m_ballSubsystem.getShootSpeed());
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
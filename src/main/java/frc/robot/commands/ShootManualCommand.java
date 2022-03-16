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
import frc.robot.subsystems.ShooterDirectionSubsystem;
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class ShootManualCommand extends CommandBase {
  private final BallShooterSubsystem m_ballSubsystem;
  private final FeedMotorSubsystem m_feedSubsystem;
  private final ShooterDirectionSubsystem m_shootDirection;
  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootManualCommand(BallShooterSubsystem ballSystem, FeedMotorSubsystem feedSystem, ShooterDirectionSubsystem shootDirection) {
    m_ballSubsystem = ballSystem;
    m_feedSubsystem = feedSystem;
    m_shootDirection = shootDirection;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ballSubsystem, m_feedSubsystem, m_shootDirection);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shootDirection.distanceToGoal() > 20){
      m_ballSubsystem.shoot(m_shootDirection.distanceToGoal() * 0.00511 - 0.01711); //calculated from linear regression
      m_feedSubsystem.start();
      if (m_feedSubsystem.ballSensor.getVoltage() > 0.75){
        m_feedSubsystem.ballFeed();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballSubsystem.stop();
    m_feedSubsystem.stop();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
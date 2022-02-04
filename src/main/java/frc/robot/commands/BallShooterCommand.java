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
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class BallShooterCommand extends CommandBase {
  private final BallShooterSubsystem m_ballsubsystem;
  private final FeedMotorSubsystem m_feedsubsystem;
  private int time;
  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BallShooterCommand(BallShooterSubsystem ballsystem, FeedMotorSubsystem feedsystem) {
    m_ballsubsystem = ballsystem;
    m_feedsubsystem = feedsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ballsubsystem, m_feedsubsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0;
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time < 25) {
      m_ballsubsystem.shoot();
      time += 1;
    }else{
      m_ballsubsystem.shoot();
      m_feedsubsystem.start();
    }
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballsubsystem.stop();
    m_feedsubsystem.stop();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
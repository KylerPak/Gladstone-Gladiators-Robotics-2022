/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.FeedMotorSubsystem;
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class BallShooterCommand extends CommandBase {
  private final BallShooterSubsystem m_ballSubsystem;
  private final FeedMotorSubsystem m_feedSubsystem;
  private XboxController m_controller = new XboxController(0);
  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BallShooterCommand(BallShooterSubsystem ballSystem, FeedMotorSubsystem feedSystem, XboxController controller) {
    m_ballSubsystem = ballSystem;
    m_feedSubsystem = feedSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ballSubsystem, m_feedSubsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getLeftTriggerAxis() > 0.05){ //Controller deadband
      m_ballSubsystem.shoot(m_controller.getLeftTriggerAxis());
      m_feedSubsystem.start();
      if(m_feedSubsystem.getVoltage() > 1){
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
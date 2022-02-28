/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterDirectionSubsystem;
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class ShooterNotLeftCommand extends CommandBase {
  private final ShooterDirectionSubsystem m_directionSubsystem;

  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterNotLeftCommand(ShooterDirectionSubsystem subsystem) {
    m_directionSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_directionSubsystem.notleft();
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_directionSubsystem.stop();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
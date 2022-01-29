/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class Autonomous extends CommandBase {
  private final DrivetrainSubsystem m_subsystem;
 
  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Autonomous(DrivetrainSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPower(0.5, 0.5);
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPower(0, 0);
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DirectionalPad;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
 
/**
 * This command will drive the robot forward for a specified period of time
 */
public class ClimbCommand extends CommandBase {
  private final ClimbSubsystem m_climbSubsystem;
  private XboxController m_controller = new XboxController(0);
  private DirectionalPad dPad = new DirectionalPad(m_controller);

  /**
   * Creates a new AutonomousCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCommand(ClimbSubsystem subsystem) {
    m_climbSubsystem = subsystem;

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
    if(dPad.up.get()){
      m_climbSubsystem.Climb();
    }
    if(dPad.down.get()){
      m_climbSubsystem.Retract();
    }
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
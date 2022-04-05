/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_subsystem;
  private XboxController m_controller = new XboxController(0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDriveCommand(DrivetrainSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setMaxOutput(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    double throttle = m_controller.getRightTriggerAxis();

    m_subsystem.tankDrive(0.6 * leftDeadband() + 0.4 * throttle, 0.55 * rightDeadband() + 0.4 * throttle);
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

  public double leftDeadband() {
    double leftValue = Math.abs(m_controller.getLeftY());
    if(leftValue < 0.06) return 0;
    else return m_controller.getLeftY() * m_controller.getLeftY() * m_controller.getLeftY();
  }

  public double rightDeadband() {
    double rightValue = Math.abs(m_controller.getRightY());
    if(rightValue < 0.06) return 0;
    else return m_controller.getRightY() * m_controller.getRightY() * m_controller.getRightY();
  }
}
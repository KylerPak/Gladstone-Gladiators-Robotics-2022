/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_subsystem;
  private XboxController m_controller = new XboxController(0);
  private final JoystickButton leftJoystickButton = new JoystickButton(m_controller, 9);

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
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if(Math.abs(m_controller.getLeftY()) < 0.05){
      m_subsystem.LeftDrive(0);
    } else{
      m_subsystem.LeftDrive(1.5 * -m_controller.getRightTriggerAxis() - 3 * m_controller.getLeftY());
    }
    if(Math.abs(m_controller.getRightY()) < 0.05){
      m_subsystem.RightDrive(0);
    } else{
      m_subsystem.RightDrive(1.5 * m_controller.getRightTriggerAxis() + 3 * m_controller.getRightY());
    }
    if(leftJoystickButton.get()){
      m_subsystem.resetEncoders();
      m_subsystem.arcadeDrive(0, 0);
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
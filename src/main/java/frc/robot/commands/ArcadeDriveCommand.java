/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
 
/**
 * An example command that uses an example subsystem.
 */
public class ArcadeDriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_subsystem;
  private XboxController m_controller = new XboxController(0);

  /**
   * Creates a new ArcadeDriveCommand.
   */
  public ArcadeDriveCommand(DrivetrainSubsystem subsystem, XboxController controller) {

    m_subsystem = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double y = m_controller.getLeftY();
    double x = m_controller.getLeftX();

    x = Math.abs(x) * x;
    y = Math.abs(y) * y;

    m_subsystem.setPower(y + x, y - x);
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
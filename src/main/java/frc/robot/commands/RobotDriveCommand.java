// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotDriveSubsystem;

public class RobotDriveCommand extends CommandBase {
  /** Creates a new RobotDriveCommand. */
  private RobotDriveSubsystem m_drive;
  private XboxController m_controller = new XboxController(0);

  public RobotDriveCommand(RobotDriveSubsystem m_robotDrive, XboxController controller) {
    this.m_drive = m_robotDrive;
    this.m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftDrive = m_controller.getLeftY();
    double rightDrive = m_controller.getRightY();
    leftDrive = Deadband(leftDrive);
    rightDrive = Deadband(rightDrive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  private double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmControl extends CommandBase {
  private ArmSubsystem m_arm;

  /** Creates a new ArmControl. */
  public ArmControl(ArmSubsystem armSystem) {
    m_arm = armSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.getPosition() < 2.45 && m_arm.getPosition() > 2.55){ //Added some deadband for give
      m_arm.armExtend();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.armRetract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeedMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualCommand extends CommandBase {
  private final IntakeSubsystem m_intakeSubsystem;
  private final FeedMotorSubsystem m_feedMotorSubsystem;
  public IntakeManualCommand(IntakeSubsystem intakesystem, FeedMotorSubsystem feedsystem) {
    m_intakeSubsystem = intakesystem;
    m_feedMotorSubsystem = feedsystem;
    addRequirements(intakesystem, feedsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.forward();
    m_feedMotorSubsystem.start();
    if(m_feedMotorSubsystem.getVoltage() > 0.75){
      m_feedMotorSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
    m_feedMotorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
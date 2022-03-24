/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AimAuto extends CommandBase {
  private final BallShooterSubsystem m_ballSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private double turnKp = 0.2;
  private double turnPower = 0;
  private double turnError = 0;

  /**
   * Creates a new LimelightAimCommand.
   */
  public AimAuto(LimelightSubsystem l, BallShooterSubsystem b) {
    this.m_limelightSubsystem = l;
    this.m_ballSubsystem = b;
    addRequirements(l);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightSubsystem.setupAim();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelightSubsystem.isTarget()) { //Target in view
      turnError = m_limelightSubsystem.getTy();
      turnPower = turnError * turnKp;
      m_ballSubsystem.rotatePower(turnPower);

      double distance = m_limelightSubsystem.getDistance();
      SmartDashboard.putNumber("Distance", distance);
    } else {
      m_ballSubsystem.rotateLeft();
      if(m_ballSubsystem.getVelocity() == 0){
        m_ballSubsystem.rotateNotleft();
      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballSubsystem.stopRotate();
    m_limelightSubsystem.setupDriveMode();
    m_ballSubsystem.contRumble();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
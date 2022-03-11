/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterDirectionSubsystem;

public class AimCommand extends CommandBase {
  private final ShooterDirectionSubsystem shooterDirection;
  private boolean isFinished;

  /**
   * Creates a new LimelightAimCommand.
   */
  public AimCommand(ShooterDirectionSubsystem shootDirection) {
    this.shooterDirection = shootDirection;

    addRequirements(shootDirection);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    shooterDirection.aiming();
    if(shooterDirection.getVelocity() == 0){
      shooterDirection.aimingLeft();
    } 
    if(shooterDirection.getHeading() == 0){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterDirection.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
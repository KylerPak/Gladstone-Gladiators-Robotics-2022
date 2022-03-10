/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterDirectionSubsystem;

public class Aim extends CommandBase {
  private final ShooterDirectionSubsystem shooterDirection;
  private final XboxController controller;
  private boolean isFinished = false;

  /**
   * Creates a new LimelightAimCommand.
   */
  public Aim(ShooterDirectionSubsystem shootDirection, XboxController controller) {
    this.shooterDirection = shootDirection;
    this.controller = controller;

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
    controller.setRumble(RumbleType.kLeftRumble, 1);
    Timer timer = new Timer();
    timer.schedule(new RumbleStopper(controller), 500);
    isFinished = true;     
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

class RumbleStopper extends TimerTask{
  private final XboxController controller;
  public RumbleStopper(XboxController controller){
    this.controller = controller;
  }
  @Override
  public void run() {
    controller.setRumble(RumbleType.kLeftRumble, 0);
  }
}
 
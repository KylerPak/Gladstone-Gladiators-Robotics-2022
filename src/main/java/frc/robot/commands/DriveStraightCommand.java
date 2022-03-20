/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightCommand extends CommandBase {
	private DrivetrainSubsystem subsystem;
  private boolean isFinished = false; 
  private Timer timer = new Timer();

  /**
   * Creates a new LimelightAimCommand.
   */
  public DriveStraightCommand(DrivetrainSubsystem subsystem) {
		this.subsystem = subsystem;
 //   addRequirements(subsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		subsystem.resetEncoders();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.tankDrive(0.7, -0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2.0);
  }

}

 
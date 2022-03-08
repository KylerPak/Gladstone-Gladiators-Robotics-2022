/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeedMotorSubsystem;
import frc.robot.subsystems.ShooterDirectionSubsystem;

public class AimandShootCommand extends CommandBase {
  private final ShooterDirectionSubsystem shooterDirection;
  private final FeedMotorSubsystem m_feedMotorSubsystem;
  private final XboxController controller;
  private double limeLightAngle = 31;
  private double limeLightHeightInches = 43.75;
  private double goalHeightInches = 104;
  private boolean isFinished = false;

  /**
   * Creates a new LimelightAimCommand.
   */
  public AimandShootCommand(ShooterDirectionSubsystem shootDirection, FeedMotorSubsystem feedSystem, XboxController controller) {
    this.shooterDirection = shootDirection;
    this.m_feedMotorSubsystem = feedSystem;
    this.controller = controller;

    addRequirements(shootDirection, feedSystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    final double Kp = -0.2; //proportional constant
    final double min_rotate = 0.05; //minimum rotation 
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ghs");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    double targetOffsetAngle_Vertical = ty.getDouble(0);
    double heading_error = tx.getDouble(0);

    if (tv.getDouble(0) < 0.5){ //target not in sight
      shooterDirection.left(); 
    } else{ //target in sight, begin aiming
      //shooterDirection.shooterDirection.set(rotate_adjust);
    }

    if (heading_error < min_rotate){ //Aiming at the target, calculating distance
      double angleToGoalDegrees = limeLightAngle + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);
      double distanceFromLimelightToGoalInches = (goalHeightInches - limeLightHeightInches)/Math.tan(angleToGoalRadians);

      if (distanceFromLimelightToGoalInches > 20){
        //shooterDirection.shooterDirection.set(distanceFromLimelightToGoalInches * 0.00511 - 0.01711); //calculated from linear regression
        m_feedMotorSubsystem.start();
        if (m_feedMotorSubsystem.ballSensor.getVoltage() > 1){
          //m_feedMotorSubsystem.ballFeed.set(0.3);
        }
        try {
          TimeUnit.SECONDS.sleep(2);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        controller.setRumble(RumbleType.kLeftRumble, 1);
        Timer timer = new Timer();
        timer.schedule(new RumbleStopper(controller), 500);
        isFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterDirection.stop();;
    m_feedMotorSubsystem.stop();
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
 
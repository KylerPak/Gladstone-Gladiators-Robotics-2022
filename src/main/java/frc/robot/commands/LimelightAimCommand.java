/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterDirectionSubsystem;

public class LimelightAimCommand extends CommandBase {
  
  private static NetworkTableEntry desiredTargetArea;
  private static NetworkTableEntry targetAngleX;
  private static NetworkTableEntry targetAngleY;
  
  private final ShooterDirectionSubsystem shooterDirection;
  private boolean isFinished = false;
  private final XboxController controller;
  /**
   * Creates a new LimelightAimCommand.
   */
  public LimelightAimCommand(ShooterDirectionSubsystem shootDirection, XboxController controller) {
    this.shooterDirection = shootDirection;
    this.controller = controller;
    final ShuffleboardTab tab = Shuffleboard.getTab("Tuning");
    if (desiredTargetArea == null) {
      desiredTargetArea =
        tab.addPersistent("Desired Target Area", 16)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("min", 0, "max", 25))
        .getEntry();
      targetAngleX = 
        tab.addPersistent("targetAngleX", 2)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("min", 0, "max", 10))
        .getEntry();
      targetAngleY = 
        tab.addPersistent("targetAngleY", 2)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("min", -5, "max", 5))
        .getEntry();
    }
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
    final double DESIRED_TARGET_AREA = desiredTargetArea.getDouble(16);        // Area of the target when the robot reaches the wall            
    double tv = NetworkTableInstance.getDefault().getTable("limelight-ghs").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight-ghs").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight-ghs").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight-ghs").getEntry("ta").getDouble(0);
    
    if (tv < 0.5){
      shooterDirection.shooterDirection.set(-0.2);
    } else if(ta < DESIRED_TARGET_AREA) {

    }
    if (ta >= DESIRED_TARGET_AREA && Math.abs(tx) <= targetAngleX.getDouble(2) && Math.abs(ty) <= targetAngleY.getDouble(5)){
      controller.setRumble(RumbleType.kLeftRumble, 1);
      Timer timer = new Timer();
      timer.schedule(new RumbleStopper(controller), 500);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterDirection.shooterDirection.set(0);
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
 
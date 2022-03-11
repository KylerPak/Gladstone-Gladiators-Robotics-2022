/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class ShooterDirectionSubsystem extends SubsystemBase {
  private static final int deviceID = Constants.shooterDirectionCANID;
  private CANSparkMax shootDirection;
  private XboxController m_controller = new XboxController(0);
  private final double kP; //Proportional Constant
  private final double allowed_error; //minimum error
  private double limeLightAngle;
  private double limeLightHeightInches;
  private double goalHeightInches;
  private double isTarget;
  private double targetOffsetAngle_Vertical;
  private double heading_error;
  private double angleToGoalDegrees;
  private double angleToGoalRadians;
  private double distanceToGoal;
  
  public ShooterDirectionSubsystem(){
    shootDirection = new CANSparkMax(deviceID, MotorType.kBrushless);
    shootDirection.restoreFactoryDefaults();
    //variables for limeLight
    limeLightAngle = 13;
    limeLightHeightInches = 44.5;
    goalHeightInches = 104;
    kP = -0.2;
    allowed_error = 0.001;
    //Soft Limits
    enableSoftLimit();
    SmartDashboard.putBoolean("Forward Soft Limit Enabled", shootDirection.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putBoolean("Reverse Soft Limit Enabled", shootDirection.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));                          
    SmartDashboard.putNumber("Forward Soft Limit", shootDirection.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Reverse Soft Limit", shootDirection.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));

  }

  public void enableSoftLimit(){
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1000);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1000);
  }

  public void left(){
    shootDirection.set(0.2);
  }
  public void stop(){
    shootDirection.set(0);
  }
  public void notleft(){
    shootDirection.set(-0.3);
  }

  public void power(double power){
    shootDirection.set(power);
  }
  public void aiming(){
    if (isTarget < 0.5){                //target not in sight
      left();
    } else{                             //target in sight, begin aiming
      power(kP * heading_error);
    }
    if (heading_error < allowed_error && heading_error != 0){
      m_controller.setRumble(RumbleType.kLeftRumble, 1);
      Timer timer = new Timer();
      timer.schedule(new RumbleStopper(m_controller), 500);
    }
  }

  public double distanceToGoal(){
    if (heading_error < allowed_error && heading_error != 0){ //Aiming at the target, calculating distance
      angleToGoalDegrees = limeLightAngle + targetOffsetAngle_Vertical;
      angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);
      distanceToGoal = (goalHeightInches - limeLightHeightInches)/Math.tan(angleToGoalRadians);
      SmartDashboard.putNumber("Distance", distanceToGoal);
    }
    return distanceToGoal;
  }

  public boolean isForwardEnabled(){
    return shootDirection.isSoftLimitEnabled(SoftLimitDirection.kForward);
  }

  public boolean isReverseEnabled(){
    return shootDirection.isSoftLimitEnabled(SoftLimitDirection.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ghs");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    isTarget = tv.getDouble(0);
    targetOffsetAngle_Vertical = ty.getDouble(0);
    heading_error = tx.getDouble(0);
    SmartDashboard.putNumber("Heading Error", heading_error);
    SmartDashboard.putNumber("Target in Sight", isTarget);
    SmartDashboard.putNumber("Target Offset", targetOffsetAngle_Vertical);
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
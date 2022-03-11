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
import com.revrobotics.RelativeEncoder;
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
  private RelativeEncoder m_encoder;
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
    m_encoder = shootDirection.getEncoder();
    //variables for limeLight
    limeLightAngle = 22;
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
    shootDirection.burnFlash();
  }

  public void enableSoftLimit(){
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 8);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -25);
  }

  public void left(){
    shootDirection.set(-0.2);
  }
  public void stop(){
    shootDirection.set(0);
  }
  public void notleft(){
    shootDirection.set(0.3);
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
  }

  public void aimingLeft(){
    if (isTarget < 0.5){                //target not in sight
      notleft();
    } else{                             //target in sight, begin aiming
      power(kP * heading_error);
    }
  }

  public double distanceToGoal(){
    if(isTarget == 1){
    angleToGoalDegrees = limeLightAngle + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);
    distanceToGoal = (goalHeightInches - limeLightHeightInches)/Math.tan(angleToGoalRadians);
    return distanceToGoal;
    } else{
      return 0;
    }
  }

  public double getHeading(){
    if(isTarget == 1 && heading_error <= allowed_error){
      m_controller.setRumble(RumbleType.kLeftRumble, 1);
      Timer timer = new Timer();
      timer.schedule(new RumbleStopper(m_controller), 500);
      return heading_error;
    } else {
      return 1; //Random value so it doesn't return 0
    }
  }

  public double getVelocity(){
    return m_encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ghs");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    isTarget = tv.getDouble(0);
    targetOffsetAngle_Vertical = -ty.getDouble(0);
    heading_error = tx.getDouble(0);
    distanceToGoal();
    SmartDashboard.putNumber("Heading Error", heading_error);
    SmartDashboard.putNumber("Target in Sight", isTarget);
    SmartDashboard.putNumber("Target Offset", targetOffsetAngle_Vertical);
    SmartDashboard.putNumber("Distance to Goal", distanceToGoal());
    SmartDashboard.updateValues();
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
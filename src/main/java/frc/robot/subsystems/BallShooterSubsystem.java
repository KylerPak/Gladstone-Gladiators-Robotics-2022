/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class BallShooterSubsystem extends SubsystemBase {
  private static final int deviceID = Constants.shooterDirectionCANID;
  private static final int ballShooterID = Constants.shooterCANID;
  private CANSparkMax shootDirection;
  private WPI_TalonFX ballShooter;
  private XboxController m_controller = new XboxController(0);
  private RelativeEncoder m_encoder;
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

  public BallShooterSubsystem() {
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
    ballShooter = new WPI_TalonFX(ballShooterID);
    SendableRegistry.setName(ballShooter, "ballShooter");
  }

  public void enableSoftLimit(){
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 8);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -25);
  }

  public void rotateLeft(){
    shootDirection.set(-0.2);
  }
  public void stopRotate(){
    shootDirection.set(0);
  }
  public void rotateNotleft(){
    shootDirection.set(0.3);
  }

  public void power(double power){
    shootDirection.set(power);
  }
  public void aiming(){
    if (isTarget < 0.5){                //target not in sight
      rotateLeft();
    } else{                             //target in sight, begin aiming
      power(kP * heading_error);
    }
    if (heading_error < allowed_error){
      m_controller.setRumble(RumbleType.kLeftRumble, 1);
      Timer timer = new Timer();
      timer.schedule(new RumbleStopper(m_controller), 500);
    }
  }

  public double distanceToGoal(){
    if(isTarget == 1){
    angleToGoalDegrees = limeLightAngle + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);
    distanceToGoal = (goalHeightInches - limeLightHeightInches)/Math.tan(-angleToGoalRadians);
    return distanceToGoal;
    } else{
      return 0;
    }
  }

  public double getVelocity(){
    return m_encoder.getVelocity();
  }

  public void shoot(double power) {
    ballShooter.set(ControlMode.PercentOutput, power);
  }

  public void stop() {
    ballShooter.set(ControlMode.PercentOutput, 0);
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
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class ShooterDirectionSubsystem extends SubsystemBase {
  private static final int deviceID = Constants.shooterDirectionCANID;
  private CANSparkMax shootDirection;
  
  public ShooterDirectionSubsystem(){
    shootDirection = new CANSparkMax(deviceID, MotorType.kBrushless);
    shootDirection.restoreFactoryDefaults();
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

  public boolean isForwardEnabled(){
    return shootDirection.isSoftLimitEnabled(SoftLimitDirection.kForward);
  }

  public boolean isReverseEnabled(){
    return shootDirection.isSoftLimitEnabled(SoftLimitDirection.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
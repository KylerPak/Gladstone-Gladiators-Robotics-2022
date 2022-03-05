/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class ShooterDirectionSubsystem extends SubsystemBase {
  private static final int deviceID = Constants.shooterDirectionCANID;
  private CANSparkMax m_shootDirection;
  
  private Boolean left = false;
  private Boolean notleft = false;
  
  public ShooterDirectionSubsystem(){
    m_shootDirection = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_shootDirection.restoreFactoryDefaults();
    //Soft Limits
    enableSoftLimit();
    SmartDashboard.putBoolean("Forward Soft Limit Enabled", m_shootDirection.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putBoolean("Reverse Soft Limit Enabled", m_shootDirection.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));                          
    SmartDashboard.putNumber("Forward Soft Limit", m_shootDirection.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Reverse Soft Limit", m_shootDirection.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
  }

  public void enableSoftLimit(){
    m_shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 100);
    m_shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -100);
  }

  public void left(){
    left = true;
    notleft = false;
  }
  public void stop(){
    left = false;
    notleft = false;
  }
  public void notleft(){
    left = false;
    notleft = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    enableSoftLimit();
    if(left == true){
      m_shootDirection.set(0.2);
    }
    else {
      m_shootDirection.set(0);
    }
    if(notleft == true){
      m_shootDirection.set(-0.3);
    }
    else {
      m_shootDirection.set(0);
    }
  }
}
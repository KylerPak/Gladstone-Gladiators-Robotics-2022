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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
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

  public BallShooterSubsystem() {
    shootDirection = new CANSparkMax(deviceID, MotorType.kBrushless);
    ballShooter = new WPI_TalonFX(ballShooterID);
    shootDirection.restoreFactoryDefaults();
    shootDirection.setIdleMode(IdleMode.kBrake);
    m_encoder = shootDirection.getEncoder();

    //Soft Limits
    enableSoftLimit();                      
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

  public void rotateLeft(){
    shootDirection.set(-0.2);
  }
  public void stopRotate(){
    shootDirection.set(0);
  }
  public void rotateNotleft(){
    shootDirection.set(0.3);
  }

  public void rotatePower(double power){
    shootDirection.set(power);
  }

  public void shoot(double power) {
    ballShooter.set(ControlMode.PercentOutput, power);
  }

  public double getVelocity(){
    return m_encoder.getVelocity();
  }

  public void contRumble(){
    m_controller.setRumble(RumbleType.kLeftRumble, 1);
    Timer timer = new Timer();
    timer.schedule(new RumbleStopper(m_controller), 500);
  }

  public void shootStop() {
    ballShooter.set(ControlMode.PercentOutput, 0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
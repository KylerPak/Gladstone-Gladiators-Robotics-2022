// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private static final int armID = Constants.armCANID;
  private CANSparkMax m_armMotor;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_armPID;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Initialize Motor
    m_armMotor = new CANSparkMax(armID, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();

    //Get Encoder and PID Controller Objects
    m_encoder = m_armMotor.getEncoder();
    m_armPID = m_armMotor.getPIDController();
    m_encoder.setPosition(0);
    m_encoder.setPositionConversionFactor(0.5);
    
    // PID Coefficients
    kP = 2;
    kI = 0;
    kD = 0.0005;
    kIz = 0;
    kFF = 0.00025;
    kMaxOutput = 1;
    kMinOutput = -1;

    //Smart Motion Coefficients
    maxVel = 150;
    minVel = 80;
    maxAcc = 60;
    allowedErr = 0.05;
    
    //Initalize PID
    m_armPID.setP(kP);
    m_armPID.setI(kI);
    m_armPID.setD(kD);
    m_armPID.setIZone(kIz);
    m_armPID.setFF(kFF);
    m_armPID.setOutputRange(kMinOutput, kMaxOutput);

    //Initialize Smart Motion
    int SmartMotionSlot = 0;
    m_armPID.setSmartMotionMaxVelocity(maxVel, SmartMotionSlot);
    m_armPID.setSmartMotionMinOutputVelocity(minVel, SmartMotionSlot);
    m_armPID.setSmartMotionMaxAccel(maxAcc, SmartMotionSlot);
    m_armPID.setSmartMotionAllowedClosedLoopError(allowedErr, SmartMotionSlot);
  }

  public double getPosition(){
    return m_encoder.getPosition();
  }

  public void armExtend(){
    m_armPID.setReference(2.1, ControlType.kSmartMotion);
  }

  public void armRetract(){
    m_armPID.setReference(0, ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


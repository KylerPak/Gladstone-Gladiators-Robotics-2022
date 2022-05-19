// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotDriveSubsystem extends SubsystemBase {
  /** Creates a new RobotDriveSubsystem. */
  private static final int leftDriveBackID = Constants.leftDriveBackCANID;
  private static final int leftDriveFrontID = Constants.leftDriveFrontCANID;
  private static final int rightDriveBackID = Constants.rightDriveBackCANID;
  private static final int rightDriveFrontID = Constants.rightDriveFrontCANID;
  private CANSparkMax m_leftDriveBack;
  private CANSparkMax m_rightDriveBack;
  private CANSparkMax m_leftDriveFront;
  private CANSparkMax m_rightDriveFront;
  private RelativeEncoder m_left;
  private RelativeEncoder m_right;
  private SparkMaxPIDController m_rightPIDController;
  private SparkMaxPIDController m_leftPIDController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public RobotDriveSubsystem() {
    //Left motors of the driveTrain
    m_leftDriveBack = new CANSparkMax(leftDriveBackID, MotorType.kBrushless);
    m_leftDriveFront = new CANSparkMax(leftDriveFrontID, MotorType.kBrushless);
    m_leftDriveBack.setInverted(true);
    m_leftDriveFront.setInverted(true);
    //Right motors of drivetrain
    m_rightDriveBack = new CANSparkMax(rightDriveBackID, MotorType.kBrushless);
    m_rightDriveFront = new CANSparkMax(rightDriveFrontID, MotorType.kBrushless);
    //Factory Reset to prevent unexpected behavior
    m_leftDriveBack.restoreFactoryDefaults();
    m_leftDriveFront.restoreFactoryDefaults();
    m_rightDriveBack.restoreFactoryDefaults();
    m_rightDriveFront.restoreFactoryDefaults();
    //Set Idle Modes
    m_leftDriveBack.setIdleMode(IdleMode.kBrake);
    m_leftDriveFront.setIdleMode(IdleMode.kBrake);
    m_rightDriveBack.setIdleMode(IdleMode.kBrake);
    m_rightDriveFront.setIdleMode(IdleMode.kBrake);
    //Encoders and PIDController
    m_right = m_rightDriveFront.getEncoder();
    m_rightPIDController = m_rightDriveFront.getPIDController();
    m_left = m_leftDriveFront.getEncoder();
    m_leftPIDController = m_leftDriveFront.getPIDController();
    //PID Values
    kP = Constants.kP;
    kI = Constants.kI;
    kD = Constants.kD;
    kIz = 300;
    kFF = 1E-5;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5600;
    //PID Configuration
    m_rightPIDController.setP(kP);
    m_rightPIDController.setI(kI);
    m_rightPIDController.setD(kD);
    m_rightPIDController.setIZone(kIz);
    m_rightPIDController.setFF(kFF);
    m_rightPIDController.setOutputRange(kMaxOutput, kMinOutput);
    m_leftPIDController.setP(kP);
    m_leftPIDController.setI(kI);
    m_leftPIDController.setD(kD);
    m_leftPIDController.setIZone(kIz);
    m_leftPIDController.setFF(kFF);
    m_leftPIDController.setOutputRange(kMaxOutput, kMinOutput);
  }
  
  public void drive(double leftSetPoint, double rightSetPoint) {
    rightSetPoint = rightSetPoint * maxRPM;
    m_rightPIDController.setReference(rightSetPoint, ControlType.kVelocity);
    m_rightDriveBack.follow(m_rightDriveFront);
    
    leftSetPoint = leftSetPoint * maxRPM;
    m_leftPIDController.setReference(leftSetPoint, ControlType.kVelocity);
    m_leftDriveBack.follow(m_leftDriveFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);

    if((p != kP)) { m_rightPIDController.setP(p); m_leftPIDController.setP(p); kP = p; }
    if((i != kI)) { m_rightPIDController.setI(i); m_leftPIDController.setI(i);kI = i; }
    if((d != kD)) { m_rightPIDController.setD(d); m_leftPIDController.setD(d);kD = d; }
    if((iz != kIz)) { m_rightPIDController.setIZone(iz); m_leftPIDController.setIZone(iz);kIz = iz; }
    if((ff != kFF)) { m_rightPIDController.setFF(ff); m_leftPIDController.setFF(ff); kFF = ff; }
  }
}

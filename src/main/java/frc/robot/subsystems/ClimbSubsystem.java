/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private static final int leftClimbID = Constants.leftClimbMotorCANID;
  private static final int rightClimbID = Constants.rightClimbMotorCANID;
  private CANSparkMax m_leftClimb;
  private CANSparkMax m_rightClimb;
  private SparkMaxPIDController m_leftPID;
  private SparkMaxPIDController m_rightPID;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

  public ClimbSubsystem() {
    // initialize motor
    m_leftClimb = new CANSparkMax(leftClimbID, MotorType.kBrushless);
    m_rightClimb = new CANSparkMax(rightClimbID, MotorType.kBrushless);

    m_leftClimb.restoreFactoryDefaults();
    m_rightClimb.restoreFactoryDefaults();

    m_rightClimb.follow(m_leftClimb);

    // initialze PID controller and encoder objects
    m_leftPID = m_leftClimb.getPIDController();
    m_rightPID = m_rightClimb.getPIDController();
    m_leftEncoder = m_leftClimb.getEncoder();
    m_rightEncoder = m_rightClimb.getEncoder();
    resetEncoder();

    m_leftEncoder.setPositionConversionFactor(0.2343);
    m_rightEncoder.setPositionConversionFactor(0.2343);

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_leftPID.setP(kP);
    m_leftPID.setI(kI);
    m_leftPID.setD(kD);
    m_leftPID.setIZone(kIz);
    m_leftPID.setFF(kFF);
    m_leftPID.setOutputRange(kMinOutput, kMaxOutput);
    
    m_rightPID.setP(kP);
    m_rightPID.setI(kI);
    m_rightPID.setD(kD);
    m_rightPID.setIZone(kIz);
    m_rightPID.setFF(kFF);
    m_rightPID.setOutputRange(kMinOutput, kMaxOutput);

    //Set Smart Motion Coefficients
    int smartMotionSlot = 0;
    m_leftPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_leftPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_leftPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_leftPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_rightPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_rightPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_rightPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_rightPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
 
    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);
  }

  public void resetEncoder(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void Climb(){
    m_leftPID.setReference(49.25, ControlType.kSmartMotion);
    m_rightPID.setReference(49.25, ControlType.kSmartMotion);
  }

  public void Retract(){
    m_leftPID.setReference(0, ControlType.kSmartMotion);
    m_rightPID.setReference(0, ControlType.kSmartMotion);
  }


  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_leftPID.setP(p); m_rightPID.setP(p); kP = p; }
    if((i != kI)) { m_leftPID.setI(i); m_rightPID.setI(i); kI = i; }
    if((d != kD)) { m_leftPID.setD(d); m_rightPID.setD(d); kD = d; }
    if((iz != kIz)) { m_leftPID.setIZone(iz); m_rightPID.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_leftPID.setFF(ff); m_rightPID.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_leftPID.setOutputRange(min, max);
      m_rightPID.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_leftPID.setSmartMotionMaxVelocity(maxV,0); 
      m_rightPID.setSmartMotionMaxVelocity(maxV, 0); maxVel = maxV; }
    if((minV != minVel)) { m_leftPID.setSmartMotionMinOutputVelocity(minV,0); 
      m_rightPID.setSmartMotionMinOutputVelocity(minVel, 0); minVel = minV; }
    if((maxA != maxAcc)) { m_leftPID.setSmartMotionMaxAccel(maxA,0);
      m_rightPID.setSmartMotionMaxAccel(maxA, 0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_leftPID.setSmartMotionAllowedClosedLoopError(allE,0);
      m_rightPID.setSmartMotionAllowedClosedLoopError(allE, 0); allowedErr = allE; }  
  }
}
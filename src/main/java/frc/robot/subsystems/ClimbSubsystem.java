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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private static final int leftClimbID = Constants.leftClimbMotorCANID;
  private static final int rightClimbID = Constants.rightClimbMotorCANID;
  private CANSparkMax m_leftClimb;
  private CANSparkMax m_rightClimb;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, setPoint, processVariable;

  public ClimbSubsystem() {
    // initialize motor
    m_leftClimb = new CANSparkMax(leftClimbID, MotorType.kBrushless);
    m_rightClimb = new CANSparkMax(rightClimbID, MotorType.kBrushless);

    m_leftClimb.restoreFactoryDefaults();
    m_rightClimb.restoreFactoryDefaults();

    m_rightClimb.follow(m_leftClimb);

    // initialze PID controller and encoder objects
    m_pidController = m_leftClimb.getPIDController();
    m_encoder = m_leftClimb.getEncoder();
    resetEncoder();

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    //Set Smart Motion Coefficients
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0);
  }
  
  public double setPoint(double setPoint){
    return setPoint;
  }

  public void Climb(){
    m_pidController.setReference(49.25, CANSparkMax.ControlType.kSmartMotion);
  }

  public void Retract(){
    m_pidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
  }


  @Override
  public void periodic() {

  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  //CAN Constants
  private static int leftClimbID = Constants.leftClimbMotorCANID;
  private static int rightClimbID = Constants.rightClimbMotorCANID;
  //Motors
  private CANSparkMax leftClimb;
  private CANSparkMax rightClimb;
  //Encoders
  private RelativeEncoder m_leftClimbEncoder;
  private RelativeEncoder m_rightClimbEncoder;
  //PID Controllers
  private SparkMaxPIDController m_leftClimbPID;
  private SparkMaxPIDController m_rightClimbPID;
  //Constants...  Constants everywhere...
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public ClimbSubsystem() {
    //Create Motors
    leftClimb = new CANSparkMax(leftClimbID, MotorType.kBrushless);
    rightClimb = new CANSparkMax(rightClimbID, MotorType.kBrushless);

    //Restore factory defaults
    leftClimb.restoreFactoryDefaults();
    rightClimb.restoreFactoryDefaults();

    //Create Encoders
    m_leftClimbEncoder = leftClimb.getEncoder();
    m_rightClimbEncoder = rightClimb.getEncoder();

    //Create PID Controllers
    m_leftClimbPID = leftClimb.getPIDController();
    m_rightClimbPID = rightClimb.getPIDController();

    //PID Coefficients
    kP = 5e-5; //subject to change
    kI = 1e-6; //subject to change
    kFF = 0.000156; //subject to change
    kD = 0;
    kIz = 0;
    maxRPM = 5700;
    maxVel = 2000;
    maxAcc = 1500;
    kMinOutput = -1;
    kMaxOutput = 1;

    //set PID's
    m_leftClimbPID.setP(kP);
    m_rightClimbPID.setP(kP);

    m_leftClimbPID.setI(kI);
    m_rightClimbPID.setI(kI);

    m_leftClimbPID.setD(kD);
    m_rightClimbPID.setD(kD);

    m_leftClimbPID.setIZone(kIz);
    m_rightClimbPID.setIZone(kIz);

    m_leftClimbPID.setFF(kFF);
    m_rightClimbPID.setFF(kFF);
  
    m_leftClimbPID.setOutputRange(kMinOutput, kMaxOutput);
    m_rightClimbPID.setOutputRange(kMinOutput, kMaxOutput);

    //Smart Motion coefficents
    int smartMotionSlot = 0;

    m_leftClimbPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_rightClimbPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);

    m_leftClimbPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_rightClimbPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);

    m_leftClimbPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_rightClimbPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

    m_leftClimbPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    m_rightClimbPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  public void climb(){ 

  }
  public void stop(){

  }
  public void reverse(){

  }

  @Override
  public void periodic() {

  }
}
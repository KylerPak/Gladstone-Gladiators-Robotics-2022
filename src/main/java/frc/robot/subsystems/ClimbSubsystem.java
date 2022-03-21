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
  private RelativeEncoder m_ClimbEncoder;
  //PID Controllers
  private SparkMaxPIDController m_ClimbPID;
  //Constants...  Constants everywhere...
  public double kP, maxRPM, maxVel, minVel, maxAcc, allowedErrkP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, gearboxRatio, topLimit, climberRest, motorRotationInches;

  public ClimbSubsystem() {
    //Initialize Motors
    leftClimb = new CANSparkMax(leftClimbID, MotorType.kBrushless);
    rightClimb = new CANSparkMax(rightClimbID, MotorType.kBrushless);
    rightClimb.follow(leftClimb);

    //Restore factory defaults
    leftClimb.restoreFactoryDefaults();
    rightClimb.restoreFactoryDefaults();

    //Initalize PID Controller and Encoders 
    m_ClimbPID = leftClimb.getPIDController();
    m_ClimbEncoder = leftClimb.getEncoder();

    //PID Coefficients
    kP = 0.0001; //subject to change
    kI = 1e-6; //subject to change
    kFF = 0.000156; //subject to change
    kD = 0;
    kIz = 0;
    kMinOutput = -1;
    kMaxOutput = 1;

    //Smart Motion Variables
    maxVel = 2000; // rpm
    maxAcc = 1500;

    //Ratios and variables
    gearboxRatio = 16;
    climberRest = 40.28; //Resting position from ground, subject to change depending on where it's mounted
    topLimit = 52.82; //maximum height of the climber from the ground, subject to change depending on where it's mounted
    
    //set PID's
    m_ClimbPID.setP(kP);
    m_ClimbPID.setP(kP);
    m_ClimbPID.setI(kI);
    m_ClimbPID.setI(kI);
    m_ClimbPID.setD(kD);
    m_ClimbPID.setD(kD);
    m_ClimbPID.setIZone(kIz);
    m_ClimbPID.setIZone(kIz);
    m_ClimbPID.setFF(kFF);
    m_ClimbPID.setFF(kFF);
    m_ClimbPID.setOutputRange(kMinOutput, kMaxOutput);
    m_ClimbPID.setOutputRange(kMinOutput, kMaxOutput);

    //SmartMotion
    int smartMotionSlot = 0;
    m_ClimbPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_ClimbPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_ClimbPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_ClimbPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    m_ClimbPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_ClimbPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_ClimbPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_ClimbPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    //Save variables on SparkMax
    leftClimb.burnFlash();
    rightClimb.burnFlash();
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
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
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, gearboxRatio, topLimit, climberRest, motorRotationInches;

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
    kP = 0.0001; //subject to change
    kI = 1e-6; //subject to change
    kFF = 0.000156; //subject to change
    kD = 0;
    kIz = 0;
    kMinOutput = -1;
    kMaxOutput = 1;

    //Ratios and variables
    gearboxRatio = 16;

    climberRest = 40.28; //Resting position from ground, subject to change depending on where it's mounted
    topLimit = 52.82; //maximum height of the climber from the ground, subject to change depending on where it's mounted
    

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
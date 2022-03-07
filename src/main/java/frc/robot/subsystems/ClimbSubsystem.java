/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
  //Feedforward
  private final ElevatorFeedforward m_eFeedforward = new ElevatorFeedforward(0.11, 1.08, 2.83, 0.11);

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
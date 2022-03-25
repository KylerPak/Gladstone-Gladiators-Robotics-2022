/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class FeedMotorSubsystem extends SubsystemBase {
  private static final int feedMotorID = Constants.feedMotorCANID;
  private static final int ballFeedID = Constants.ballFeedCANID;
  private WPI_TalonFX feedMotor;
  private CANSparkMax ballFeed;
  public final AnalogInput ballSensor;

  public FeedMotorSubsystem() {
    feedMotor = new WPI_TalonFX(feedMotorID);
    ballFeed = new CANSparkMax(ballFeedID, MotorType.kBrushless);  
    ballSensor = new AnalogInput(0);
  }

  public void intakeFeed(){
    feedMotor.set(ControlMode.PercentOutput, 0.35);
    if(getVoltage() > 0.75){
      stop();
    }
  }

  public void feedBall(){
    feedMotor.set(ControlMode.PercentOutput, 0.35);
    if(getVoltage() > 0.75){
      ballFeed();
    }
  }

  public void start() {
    feedMotor.set(ControlMode.PercentOutput, 0.35);
  }

  public void stop() {
    feedMotor.set(0);
    ballFeed.set(0);
  }

  public void reverse(){
    feedMotor.set(ControlMode.PercentOutput, -0.2);
    ballFeed.set(-0.2);
  }

  public void ballFeed(){
    ballFeed.set(0.3);
  }

  public double getVoltage(){
    return ballSensor.getVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double sensorVoltage = getVoltage();
    SmartDashboard.putNumber("Sensor Voltage", sensorVoltage);

  }
}
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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private RelativeEncoder m_rotationEncoder;

  private final BangBangController shootController;
  private final SimpleMotorFeedforward shootFeedForward;

  private double kS = 1;
  private double kV = 1;
  private double kA = 1;

  private final int kFalconUnitsPerRev = 2048; //Falcon 500 integrated sensor rate
  private final NeutralMode kBrakeDuringNeutral = NeutralMode.Coast;

  public BallShooterSubsystem() {
    shootDirection = new CANSparkMax(deviceID, MotorType.kBrushless);  
    shootDirection.restoreFactoryDefaults();
    shootDirection.setIdleMode(IdleMode.kBrake);
    m_rotationEncoder = shootDirection.getEncoder();

    ballShooter = new WPI_TalonFX(ballShooterID);
    ballShooter.setNeutralMode(kBrakeDuringNeutral);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    ballShooter.configAllSettings(configs);

    shootController = new BangBangController();
    shootFeedForward = new SimpleMotorFeedforward(kS, kV, kA);

    //Soft Limits
    enableSoftLimit();                      
    SmartDashboard.putNumber("Forward Soft Limit", shootDirection.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Reverse Soft Limit", shootDirection.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
    shootDirection.burnFlash();
  }

  public void enableSoftLimit(){
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    shootDirection.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    shootDirection.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -15);
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
    return m_rotationEncoder.getVelocity();
  }

  public double getShootSpeed(){
    return ballShooter.getSelectedSensorVelocity(0) / 2048 * 1.5; //speed of shooterWheels per 100ms
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
    getShootSpeed();
    SmartDashboard.putNumber("Shooter Speed", getShootSpeed());
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
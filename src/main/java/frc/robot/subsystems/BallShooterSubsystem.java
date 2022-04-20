/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.linearInterpolator;
 
public class BallShooterSubsystem extends SubsystemBase {
  private static final int deviceID = Constants.shooterDirectionCANID;
  private static final int ballShooterID = Constants.shooterCANID;
  private CANSparkMax shootDirection;
  private WPI_TalonFX ballShooter;
  private RelativeEncoder m_rotationEncoder;

  private double shootSetPoint = 15; //Default value for default shooting
  private double shootPower = 0;


  private final BangBangController shootController;
  private final SimpleMotorFeedforward shootFeedForward;

  private double kS = 0.95696;
  private double kV = 0.016912;
  private double kA = 0.0045713;

  private final NeutralMode kCoastDuringNeutral = NeutralMode.Coast;

  private double [][] shooterData = {
    {5.0, 15},
    {8.0, 20},
    {11.0, 25},
    {14.0, 30}
  };

  private final linearInterpolator shootInterpolator = new linearInterpolator(shooterData);

  public BallShooterSubsystem() {
    shootDirection = new CANSparkMax(deviceID, MotorType.kBrushless);  
    shootDirection.restoreFactoryDefaults();
    shootDirection.setIdleMode(IdleMode.kBrake);
    m_rotationEncoder = shootDirection.getEncoder();

    ballShooter = new WPI_TalonFX(ballShooterID);
    ballShooter.setNeutralMode(kCoastDuringNeutral);
    ballShooter.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 1));
    ballShooter.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.5));
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    ballShooter.configAllSettings(configs);

    SmartDashboard.putNumber("Shoot Target Speed", shootSetPoint);

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

  public void shoot(){
    shootSetPoint = SmartDashboard.getNumber("Shoot Target Speed", shootSetPoint);

    double bangPower = shootController.calculate(shootSetPoint);
    double velPerSecond = getShootSpeed() * 10; //speed of shooterWheels per 1 second
    double feedForwardPower = 0.9 * shootFeedForward.calculate(velPerSecond, velPerSecond*2);

    shootPower = bangPower + feedForwardPower;
    ballShooter.set(ControlMode.PercentOutput, shootPower);
  }

  public void shootAtDistance() {
    ballShooter.set(ControlMode.PercentOutput, shootSetPoint);
  }

  public double getVelocity(){
    return m_rotationEncoder.getVelocity();
  }

  public double getShootSpeed(){
    return ballShooter.getSelectedSensorVelocity(0) / 2048 * 1.5; //speed of shooterWheels per 100ms
  }

  public void shootStop() {
    ballShooter.set(ControlMode.PercentOutput, 0);
  }
  
  public void setSetPoints(double s){
    shootSetPoint = s;
  }

  public void setPointsFromDistance(double distance){
    double distanceinFeet = distance / 12;
    double shooterSpeed = shootInterpolator.getInterpolatedValue(distanceinFeet);

    setSetPoints(shooterSpeed);
  }

  public boolean atTargetVelocity(){
    return getShootSpeed() > shootSetPoint;
  }

  public void shootDistance(){
    ballShooter.set(ControlMode.PercentOutput, 0.65);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double OutPutCurrent = ballShooter.getStatorCurrent();
    double SupplyCurrent = ballShooter.getSupplyCurrent();
    SmartDashboard.putNumber("Output Current", OutPutCurrent);
    SmartDashboard.putNumber("Supply Current", SupplyCurrent);
  }
}
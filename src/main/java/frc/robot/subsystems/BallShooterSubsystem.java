/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class BallShooterSubsystem extends SubsystemBase {
  private WPI_TalonFX ballShooter;
  private static final int ballShooterID = Constants.shooterCANID;
  public BallShooterSubsystem() {
    ballShooter = new WPI_TalonFX(ballShooterID);
    SendableRegistry.setName(ballShooter, "ballShooter");
  }

  public void shoot(double power) {
    ballShooter.set(ControlMode.PercentOutput,power);
  }

  public void stop() {
    ballShooter.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
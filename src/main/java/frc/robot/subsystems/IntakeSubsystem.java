/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class IntakeSubsystem extends SubsystemBase {
  private static final int intakeMotorID = Constants.intakeMotorCANID;
  private WPI_TalonFX intakeMotor;
  
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonFX(intakeMotorID);
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 19, 0.5));
    TalonFXConfiguration configs = new TalonFXConfiguration();
    intakeMotor.configAllSettings(configs);
  }

  public void forward() {
    intakeMotor.set(ControlMode.PercentOutput, 0.35);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reverse(){
    intakeMotor.set(ControlMode.PercentOutput, -0.35);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
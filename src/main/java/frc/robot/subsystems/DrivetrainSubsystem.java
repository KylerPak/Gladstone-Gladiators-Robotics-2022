/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class DrivetrainSubsystem extends SubsystemBase {
  private final Talon leftDriveFront = new Talon(1); 
  private final Talon leftDriveBack = new Talon(3); 
  private final Talon rightDriveFront = new Talon(2); 
  private final Talon rightDriveBack = new Talon(4); 
  private final MotorController rightMotor = new MotorControllerGroup(rightDriveFront, rightDriveBack);
  public double speed;
  public double rotation;
  
  public void setPower(double leftPower, double rightPower) {
    leftDriveFront.set(leftPower);
    leftDriveBack.set(-leftPower);
    rightMotor.set(-rightPower);
  } 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
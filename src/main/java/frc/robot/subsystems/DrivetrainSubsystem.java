/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class DrivetrainSubsystem extends SubsystemBase {
  public WPI_TalonSRX leftDriveFront = new WPI_TalonSRX(Constants.leftDriveFrontCANID); 
  public WPI_TalonSRX leftDriveBack = new WPI_TalonSRX(Constants.leftDriveBackCANID);
  public WPI_VictorSPX rightDriveFront = new WPI_VictorSPX(Constants.rightDriveFrontCANID); 
  public WPI_TalonSRX rightDriveBack = new WPI_TalonSRX(Constants.rightDriveBackCANID); 
  private final MotorController rightMotor = new MotorControllerGroup(rightDriveFront, rightDriveBack);
  private Encoder m_rightEncoder = new Encoder(0, 1);
  private Encoder m_leftEncoder = new Encoder(2, 3);
  //public double speed;
  //public double rotation;

  public DrivetrainSubsystem() {

  }
  
  public void setPower(double leftPower, double rightPower) {
    leftDriveFront.set(leftPower);
    leftDriveBack.set(-leftPower);
    rightMotor.set(-rightPower);
  }
  
  public Encoder getLeftEncoder() {
    m_leftEncoder.reset();
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    m_rightEncoder.reset();
    return m_leftEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
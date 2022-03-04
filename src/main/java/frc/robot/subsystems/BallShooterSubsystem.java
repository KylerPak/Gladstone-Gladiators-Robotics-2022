/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class BallShooterSubsystem extends SubsystemBase {
  private WPI_TalonFX ballShooter = new WPI_TalonFX(Constants.shooterCANID);
  private WPI_TalonSRX ballFeed = new WPI_TalonSRX(Constants.ballFeedCANID);
  private Boolean isshooting = false;
  public BallShooterSubsystem() {
    SendableRegistry.setName(ballShooter, "ballShooter");
  }

  public void shoot() {
    isshooting = true;
  }

  public void stop() {
    isshooting = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isshooting == true){
      ballShooter.set(-1);
      ballFeed.set(1);
    }
    else {
      ballShooter.set(0);
      ballFeed.set(0);
    }
  }
}
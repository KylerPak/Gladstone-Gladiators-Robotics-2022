/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class FeedMotorSubsystem extends SubsystemBase {
  public WPI_TalonFX feedMotor = new WPI_TalonFX(Constants.FeedMotorCANID);
  private Boolean running = false;
  public FeedMotorSubsystem() {
    SendableRegistry.setName(feedMotor, "feedMotor");
  }

  public void start() {
    running = true;
  }

  public void stop() {
    running = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(running == true){
      feedMotor.set(-1);
    }
    else {
      feedMotor.set(0);
    }
  }
}
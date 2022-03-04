/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class FeedMotorSubsystem extends SubsystemBase {
  public WPI_VictorSPX feedMotor = new WPI_VictorSPX(Constants.feedMotorCANID);
  public WPI_TalonSRX ballFeed = new WPI_TalonSRX(Constants.ballFeedCANID);
  public final DigitalInput ballSensor = new DigitalInput(9);
  private Boolean running = false;
  private Boolean reverse = false;
  public FeedMotorSubsystem() {
    SendableRegistry.setName(feedMotor, "feedMotor");
  }

  public void start() {
    running = true;
    reverse = false;
  }

  public void stop() {
    running = false;
    reverse = false;
  }

  public void reverse(){
    running = false;
    reverse = true;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(running == true){
      feedMotor.set(-1);
    }
    else{
      feedMotor.set(0);
    }
    if(reverse == true){
      feedMotor.set(1);
    }
    else {
      feedMotor.set(0);
    }
  }
}
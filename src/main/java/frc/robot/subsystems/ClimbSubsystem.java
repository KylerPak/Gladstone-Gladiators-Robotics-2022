/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private WPI_VictorSPX climbMotor = new WPI_VictorSPX(Constants.climbMotorCANID); 
  private boolean running = false;

  public ClimbSubsystem() {
    SendableRegistry.setName(climbMotor, "climbMotor");
  }

  public void climb(){
    running = true;
  }
  public void stop(){
    running = false;
  }
  public void reverse(){
    climbMotor.set(0.3);
  }

  @Override
  public void periodic() {
    if(running == true){
      climbMotor.set(-0.75);
    } else{
      climbMotor.set(0);
    }
  }
}
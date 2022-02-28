/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
public class ShooterDirectionSubsystem extends SubsystemBase {
  public WPI_TalonSRX shooterDirection = new WPI_TalonSRX(Constants.shooterDirectionCANID);
  private Boolean left = false;
  private Boolean notleft = false;
  
  public ShooterDirectionSubsystem(){
    SendableRegistry.setName(shooterDirection, "shooterDirection");
  }

  public void left(){
    left = true;
    notleft = false;
  }
  public void stop(){
    left = false;
    notleft = false;
  }
  public void notleft(){
    left = false;
    notleft = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(left == true){
      shooterDirection.set(-0.2);
    }
    else {
      shooterDirection.set(0);
    }
    if(notleft == true){
      shooterDirection.set(0.2);
    }
    else {
      shooterDirection.set(0);
    }
  }
}
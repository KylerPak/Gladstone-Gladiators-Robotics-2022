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
 
public class IntakeSubsystem extends SubsystemBase {
  private static final int intakeMotorID = Constants.intakeMotorCANID;
  private WPI_TalonFX intakeMotor;
  private Boolean running = false;
  private Boolean reverse = false;
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonFX(intakeMotorID);
    SendableRegistry.setName(intakeMotor, "intakeMotor");
  }

  public void forward() {
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
      intakeMotor.set(-1);
    }
    else {
      intakeMotor.set(0);
    }
    if(reverse == true){
      intakeMotor.set(1);
    }
    else {
      intakeMotor.set(0);
    }
  }
}
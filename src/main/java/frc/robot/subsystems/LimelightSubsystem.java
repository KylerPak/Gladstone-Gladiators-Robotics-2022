// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable table;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight-ghs");
      setPipeline(0);
      setupDriveMode();
  }

  public boolean isTarget(){
    return getValue("tv").getDouble(0) == 1;
  }

  public double getTx(){
    return getValue("tx").getDouble(0);
  }

  public double getTy(){
    return getValue("ty").getDouble(0);
  }

  public void setLedMode(int ledMode) {
		getValue("ledMode").setNumber(ledMode);
  	}
  
	public void setCameraMode(int cameraMode) {
		getValue("camMode").setNumber(cameraMode);
 	}

  public void setupAim(){
    setLedMode(3);
    setCameraMode(0);
  }

  public void setupDriveMode(){
    setLedMode(1);
    setCameraMode(0);
  }

  public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

  private NetworkTableEntry getValue(String key){
    return table.getEntry(key);
  }

  public double getDistance(){
    return (104 - 25) / Math.tan(Math.toRadians(22 + getTx()));
  }
}
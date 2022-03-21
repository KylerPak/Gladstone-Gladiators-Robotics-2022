// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class ClimbSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax m_leftClimb = new CANSparkMax(Constants.leftClimbMotorCANID, MotorType.kBrushless);
  private final CANSparkMax m_rightClimb = new CANSparkMax(Constants.rightClimbMotorCANID, MotorType.kBrushless);
  private final RelativeEncoder m_ClimbEncoder = m_leftClimb.getEncoder();
  private final ElevatorFeedforward m_climbFeedforward = new ElevatorFeedforward(0.4, 0.33, 32.74, 0.03);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            2,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1.75, 0.75)));
    m_rightClimb.follow(m_leftClimb);
    m_ClimbEncoder.setPositionConversionFactor(0.2343); //inches per rotation
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedForward = m_climbFeedforward.calculate(setpoint.position, setpoint.velocity);

    // Use the output (and optionally the setpoint) here
    m_leftClimb.setVoltage(output + feedForward);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_ClimbEncoder.getPosition(); //In inches
  }
}

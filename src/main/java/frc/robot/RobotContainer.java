/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DrivetrainSubsystem m_driveTrainSubsystem = new DrivetrainSubsystem();
  private final FeedMotorSubsystem m_feedMotorSubsystem = new FeedMotorSubsystem();
  private final BallShooterSubsystem m_ballShooterSubsystem = new BallShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  public final Autonomous m_autoCommand;
  public final BallShooterCommand m_ballShooterCommand;
  private final ClimbCommand m_climbCommand;
  private final ClimbReverseCommand m_climbReverse;
  private final XboxController m_controller = new XboxController(0);
  private final JoystickButton aButton = new JoystickButton(m_controller, 1);
  //private final JoystickButton bButton = new JoystickButton(m_controller, 2);
  //private final JoystickButton xButton = new JoystickButton(m_controller, 3);
  //private final JoystickButton yButton = new JoystickButton(m_controller, 4);
  //private final JoystickButton leftBumper = new JoystickButton(m_controller, 5);
  private final JoystickButton rightBumper = new JoystickButton(m_controller, 6);
  //private final JoystickButton leftMiddleButton = new JoystickButton(m_controller, 7);
  //private final JoystickButton rightMiddleButton = new JoystickButton(m_controller, 8);
  private DirectionalPad dPad = new DirectionalPad(m_controller);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driveTrainSubsystem.setDefaultCommand(new TeleopDriveCommand(m_driveTrainSubsystem, m_controller));

    m_autoCommand = new Autonomous(m_driveTrainSubsystem);
    m_ballShooterCommand = new BallShooterCommand(m_ballShooterSubsystem, m_feedMotorSubsystem);
    m_climbCommand = new ClimbCommand(m_climbSubsystem);
    m_climbReverse = new ClimbReverseCommand(m_climbSubsystem);
    // Configure the button bindings
    configureButtonBindings();
  }
  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoysticpixyLightskButton}.
   */
  private void configureButtonBindings() {
    aButton.whenPressed(m_autoCommand);
    rightBumper.whenHeld(m_ballShooterCommand);
    dPad.up.toggleWhenActive(m_climbCommand);
    dPad.down.toggleWhenActive(m_climbReverse);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }*/


}
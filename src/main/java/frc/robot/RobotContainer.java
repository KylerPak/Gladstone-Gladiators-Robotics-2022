/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DrivetrainSubsystem m_driveTrainSubsystem;
  private final FeedMotorSubsystem m_feedSubsystem;
  private final BallShooterSubsystem m_ballShooterSubsystem;
  private final ClimbSubsystem m_climbSubsystem;
  private final ShooterDirectionSubsystem m_directionSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  public final AimCommand m_Aim;
  public final BallShooterCommand m_ballShoot;
  public final SequentialCommandGroup autoCommand;
  private final ClimbCommand m_climbCommand;
  private final ClimbReverseCommand m_climbReverse;
  private final IntakeCommand m_intakeCommand;
  private final IntakeReverseCommand m_intakeReverse;
  private final FeedMotorReverseCommand m_feedmotorReverse;
  private final ShooterLeftCommand m_shooterLeft;
  private final ShooterNotLeftCommand m_shooterNotLeft;
  private final XboxController m_controller = new XboxController(0);
  //private final JoystickButton aButton = new JoystickButton(m_controller, 1);
  //private final JoystickButton bButton = new JoystickButton(m_controller, 2);
  private final JoystickButton xButton = new JoystickButton(m_controller, 3);
  //private final JoystickButton yButton = new JoystickButton(m_controller, 4);
  private final JoystickButton leftBumper = new JoystickButton(m_controller, 5);
  private final JoystickButton rightBumper = new JoystickButton(m_controller, 6);
  private final JoystickButton leftMiddleButton = new JoystickButton(m_controller, 7);
  private final JoystickButton rightMiddleButton = new JoystickButton(m_controller, 8);
  private DirectionalPad dPad = new DirectionalPad(m_controller);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driveTrainSubsystem = new DrivetrainSubsystem();
    m_feedSubsystem = new FeedMotorSubsystem();
    m_ballShooterSubsystem = new BallShooterSubsystem();
    m_directionSubsystem = new ShooterDirectionSubsystem();
    m_climbSubsystem = new ClimbSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();

    m_driveTrainSubsystem.setDefaultCommand(new TeleopDriveCommand(m_driveTrainSubsystem, m_controller));

    m_Aim = new AimCommand(m_directionSubsystem);
    m_ballShoot = new BallShooterCommand(m_ballShooterSubsystem, m_feedSubsystem, m_directionSubsystem);
    m_climbCommand = new ClimbCommand(m_climbSubsystem);
    m_climbReverse = new ClimbReverseCommand(m_climbSubsystem);
    m_intakeCommand = new IntakeCommand(m_intakeSubsystem, m_feedSubsystem);
    m_intakeReverse = new IntakeReverseCommand(m_intakeSubsystem);
    m_feedmotorReverse = new FeedMotorReverseCommand(m_feedSubsystem);
    m_shooterLeft = new ShooterLeftCommand(m_directionSubsystem);
    m_shooterNotLeft = new ShooterNotLeftCommand(m_directionSubsystem);

    autoCommand = new SequentialCommandGroup(m_Aim, m_ballShoot); 
    //SendableChooser
    m_chooser.setDefaultOption("Aim and Shoot", autoCommand);
    SmartDashboard.putData(m_chooser);
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
    leftBumper.whenHeld(m_intakeCommand);
    rightBumper.whenActive(m_ballShoot);
    leftMiddleButton.whenHeld(m_intakeReverse);
    rightMiddleButton.whenHeld(m_feedmotorReverse);
    xButton.whenPressed(m_Aim);
    dPad.up.whenHeld(m_climbCommand);
    dPad.down.whenHeld(m_climbReverse);
    dPad.left.whenHeld(m_shooterLeft);
    dPad.right.whenHeld(m_shooterNotLeft);
  }

public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
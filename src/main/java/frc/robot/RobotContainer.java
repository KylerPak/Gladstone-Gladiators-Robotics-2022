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
  private final IntakeSubsystem m_intakeSubsystem;
  private final AimCommand m_Aim;
  private final AimManual m_aimManual;
  private final BallShooterCommand m_ballShoot;
  private final PathWeaverCommand m_pathWeaver;
  private final DoNothingCommand m_Nothing;
  private final IntakeCommand m_intakeCommand;
  private final IntakeManualCommand m_intakeManual;
  private final IntakeReverseCommand m_intakeReverse;
  private final IntakeStop m_intakeStop;
  private final FeedMotorReverseCommand m_feedmotorReverse;
  //private final ShooterNotLeftCommand m_shooterNotLeft;
  private final ShootManualCommand m_shootManual;
  private final ShootAtDistance m_atDistance;
  private final DriveStraightCommand m_straight;
  private final SequentialCommandGroup autoCommand;
  private final XboxController m_controller = new XboxController(0);
  private final JoystickButton aButton = new JoystickButton(m_controller, 1);
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
    m_climbSubsystem = new ClimbSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();

    m_driveTrainSubsystem.setDefaultCommand(new TeleopDriveCommand(m_driveTrainSubsystem, m_controller));
    m_ballShooterSubsystem.setDefaultCommand(new ShooterRotate(m_ballShooterSubsystem));
    m_climbSubsystem.setDefaultCommand(new ClimbCommand(m_climbSubsystem));

    m_Aim = new AimCommand(m_ballShooterSubsystem);
    m_aimManual = new AimManual(m_ballShooterSubsystem);
    m_ballShoot = new BallShooterCommand(m_ballShooterSubsystem, m_feedSubsystem);
    m_shootManual = new ShootManualCommand(m_ballShooterSubsystem, m_feedSubsystem);
    m_atDistance = new ShootAtDistance(m_ballShooterSubsystem, m_feedSubsystem);
    m_intakeCommand = new IntakeCommand(m_intakeSubsystem, m_feedSubsystem);
    m_intakeReverse = new IntakeReverseCommand(m_intakeSubsystem);
    m_intakeManual = new IntakeManualCommand(m_intakeSubsystem, m_feedSubsystem);
    m_intakeStop = new IntakeStop(m_intakeSubsystem, m_feedSubsystem);
    m_feedmotorReverse = new FeedMotorReverseCommand(m_feedSubsystem);
    m_Nothing = new DoNothingCommand();
    m_straight = new DriveStraightCommand(m_driveTrainSubsystem);
    m_pathWeaver = new PathWeaverCommand(m_driveTrainSubsystem);
    autoCommand = new SequentialCommandGroup(m_intakeCommand, m_pathWeaver, m_intakeStop, m_Aim, m_ballShoot); 
    //SendableChooser
    m_chooser.setDefaultOption("Aim and Shoot", autoCommand);
    m_chooser.addOption("Do Nothing", m_Nothing);
    m_chooser.addOption("Drive Striaght", m_straight);
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
    leftBumper.whenHeld(m_intakeManual);
    rightBumper.whenHeld(m_atDistance);
    leftMiddleButton.whenHeld(m_intakeReverse);
    rightMiddleButton.whenHeld(m_feedmotorReverse);
    aButton.whenHeld(m_shootManual);
    xButton.whenPressed(m_aimManual);
  }

public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
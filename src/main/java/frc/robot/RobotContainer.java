/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commands.AutonomousCommands.Autonomous;
import frc.robot.commands.AutonomousCommands.DoNothingCommand;
import frc.robot.commands.AutonomousCommands.DriveStraightCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final FeedMotorSubsystem m_feedSubsystem = new FeedMotorSubsystem();
  private final BallShooterSubsystem m_ballShooterSubsystem = new BallShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final Autonomous m_auto = new Autonomous(m_limelight, m_driveTrainSubsystem, m_armSubsystem, m_intakeSubsystem, m_ballShooterSubsystem, m_feedSubsystem);
  private final DoNothingCommand m_Nothing = new DoNothingCommand();
  private final DriveStraightCommand m_straight = new DriveStraightCommand(m_driveTrainSubsystem, m_armSubsystem, m_limelight, m_ballShooterSubsystem, m_feedSubsystem);
  private final XboxController m_controller = new XboxController(0);
  //private final JoystickButton aButton = new JoystickButton(m_controller, 1);
  private final JoystickButton bButton = new JoystickButton(m_controller, 2);
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

    m_driveTrainSubsystem.setDefaultCommand(new TeleopDriveCommand(m_driveTrainSubsystem, m_controller));
    m_armSubsystem.setDefaultCommand(new ArmControl(m_armSubsystem));

    //SendableChooser
    m_chooser.setDefaultOption("3 Ball Auto", m_auto);
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
    //Intake
    leftBumper.whileHeld(
      new ParallelCommandGroup(
        new RunCommand(m_intakeSubsystem::forward, m_intakeSubsystem),
        new RunCommand(m_feedSubsystem::intakeFeed, m_feedSubsystem)));
    leftBumper.whenReleased(
      new ParallelCommandGroup(
        new RunCommand(m_intakeSubsystem::stop, m_intakeSubsystem),
        new RunCommand(m_feedSubsystem::stop, m_feedSubsystem)));
    //Shooting
    rightBumper.whileHeld(
      new ShootAtDistance(m_ballShooterSubsystem, m_feedSubsystem));
    rightBumper.whenReleased(
      new ParallelCommandGroup(
        new RunCommand(m_intakeSubsystem::stop, m_intakeSubsystem),
        new RunCommand(m_feedSubsystem::stop, m_feedSubsystem)));
    //Intake Reverse
    leftMiddleButton.whileHeld(
      new ParallelCommandGroup(
        new RunCommand(m_intakeSubsystem::reverse, m_intakeSubsystem),
        new RunCommand(m_feedSubsystem::reverse, m_feedSubsystem)));
    leftMiddleButton.whenReleased(
      new ParallelCommandGroup(
        new RunCommand(m_intakeSubsystem::stop, m_intakeSubsystem),
        new RunCommand(m_feedSubsystem::stop, m_feedSubsystem)));
    //Feed Reverse
    rightMiddleButton.whileHeld(
      new RunCommand(m_feedSubsystem::reverse, m_feedSubsystem));
    rightMiddleButton.whenReleased(
      new RunCommand(m_feedSubsystem::stop, m_feedSubsystem));
    //Limelight
    xButton.whenHeld(new AimAuto(m_limelight, m_ballShooterSubsystem));
    //Climber
    dPad.up.whenPressed(new ClimbExtend(m_climbSubsystem));
    dPad.down.whenPressed(new ClimbRetract(m_climbSubsystem));
    //Turret Rotate
    dPad.left.whileHeld(
      new RunCommand(m_ballShooterSubsystem::rotateLeft, m_ballShooterSubsystem));
    dPad.left.whenReleased(
      new RunCommand(m_ballShooterSubsystem::stopRotate, m_ballShooterSubsystem));
    dPad.right.whileHeld(
      new RunCommand(m_ballShooterSubsystem::rotateNotleft, m_ballShooterSubsystem));
    dPad.right.whenReleased(
      new RunCommand(m_ballShooterSubsystem::stopRotate, m_ballShooterSubsystem));
    bButton.whenPressed(new ArmRetract(m_armSubsystem));
  }

public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
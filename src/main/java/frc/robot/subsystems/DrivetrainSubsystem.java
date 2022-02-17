package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  //Left Side Motors
  public WPI_TalonSRX leftDriveFront = new WPI_TalonSRX(Constants.leftDriveFrontCANID); 
  public WPI_TalonSRX leftDriveBack = new WPI_TalonSRX(Constants.leftDriveBackCANID);
  private final MotorController m_leftMotors = new MotorControllerGroup(leftDriveFront, leftDriveBack);
  //Right Side Motors
  public WPI_TalonSRX rightDriveFront = new WPI_TalonSRX(Constants.rightDriveFrontCANID); 
  public WPI_TalonSRX rightDriveBack = new WPI_TalonSRX(Constants.rightDriveBackCANID); 
  private final MotorController m_rightMotors = new MotorControllerGroup(rightDriveFront, rightDriveBack);
  //Robot Drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  //Encoders
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);
  //Gyro Sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();
  //Odometry
  private final DifferentialDriveOdometry m_odometry;
  //PID Controller
  private final PIDController leftPID = new PIDController(3.38, 0, 0);
  private final PIDController rightPID = new PIDController(3.38, 0, 0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
  //Encoder Constants
  private static final double whd = 6; //wheel count
  private static final double cpr = 64; //counts per revolution
  //Constants for UniversalDriveTrain
  public double speed;
  public double rotation;
  //Extra variables needed
  public Pose2d pose;

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages for both sides to move forward
    m_rightMotors.setInverted(true);
    
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Math.PI*whd/cpr);
    m_rightEncoder.setDistancePerPulse(Math.PI*whd/cpr);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(getHeading());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block and updates the pose
    pose = m_odometry.update(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param speed the commanded forward movement
   * @param rotation the commanded rotation
   */
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void VoltageDrive(double leftVelocitySetpoint, double rightVelocitySetpoint) {
    m_leftMotors.setVoltage(feedforward.calculate(leftVelocitySetpoint)
      + leftPID.calculate(m_leftEncoder.getRate(), leftVelocitySetpoint));
    m_rightMotors.setVoltage(feedforward.calculate(rightVelocitySetpoint)
      + rightPID.calculate(m_rightEncoder.getRate(), rightVelocitySetpoint)); 
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  //Return the feedforward of the robot
  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  //Return leftPID
  public PIDController getLeftPidController(){
    return getLeftPidController();
  }

  //Return RightPID
  public PIDController getRightPidController(){
    return getRightPidController();
  }
}
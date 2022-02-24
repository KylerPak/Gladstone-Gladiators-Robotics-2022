package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  //Talon Motors
  private WPI_TalonSRX leftDriveFront = new WPI_TalonSRX(Constants.leftDriveFrontCANID);
  private WPI_TalonSRX rightDriveFront = new WPI_TalonSRX(Constants.rightDriveFrontCANID);
  //NEO Motors 
  private CANSparkMax leftDriveBack = new CANSparkMax(Constants.leftDriveBackCANID, MotorType.kBrushless);
  private CANSparkMax rightDriveBack = new CANSparkMax(Constants.rightDriveBackCANID, MotorType.kBrushless);
  //Motor Controllers
  private MotorController m_leftMotors = new MotorControllerGroup(leftDriveFront, leftDriveBack);
  private MotorController m_rightMotors = new MotorControllerGroup(rightDriveFront, rightDriveBack);
  //DriveTrain
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  //Encoders
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  //Odometry and Gyro
  private DifferentialDriveOdometry m_odometry;
  private Gyro m_gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
  //Feedforward and PID Controller
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksVolts, 
    Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
  private PIDController leftPID = new PIDController(3.38, 0, 0);
  private PIDController rightPID = new PIDController(3.38, 0, 0);
  //Variables needed
  public Pose2d pose;
  public double speed;
  public double rotation;


  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
   leftDriveBack.restoreFactoryDefaults();
   rightDriveBack.restoreFactoryDefaults();

   m_leftEncoder = leftDriveBack.getEncoder();
   m_rightEncoder = rightDriveBack.getEncoder();
   m_odometry = new DifferentialDriveOdometry(getHeading());

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block and updates the pose
    pose = m_odometry.update(
      m_gyro.getRotation2d(), 
      m_leftEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(3) / 60, //speed of leftwheels in meters per second 
      m_rightEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(3) / 60 //speed of rightwheels in meters per second
      );
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
      + leftPID.calculate(m_leftEncoder.getVelocity(), leftVelocitySetpoint));
    m_rightMotors.setVoltage(feedforward.calculate(rightVelocitySetpoint)
      + rightPID.calculate(m_rightEncoder.getVelocity(), rightVelocitySetpoint)); 
    m_drive.feed();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public PIDController getLeftPID(){
    return leftPID;
  }

  public PIDController getRightPID(){
    return rightPID;
  }

  public RelativeEncoder getLeftEncoder(){
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder(){
    return m_rightEncoder;
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }


}
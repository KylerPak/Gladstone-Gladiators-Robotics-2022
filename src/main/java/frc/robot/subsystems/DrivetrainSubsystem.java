package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  //Motors and ID's
  private static final int leftDriveBackID = Constants.leftDriveBackCANID;
  private static final int leftDriveFrontID = Constants.leftDriveFrontCANID;
  private static final int rightDriveBackID = Constants.rightDriveBackCANID;
  private static final int rightDriveFrontID = Constants.rightDriveFrontCANID;
  private CANSparkMax m_leftDriveBack;
  private CANSparkMax m_rightDriveBack;
  private CANSparkMax m_leftDriveFront;
  private CANSparkMax m_rightDriveFront;
  //Differential Drive and Kinematics
  private DifferentialDrive m_drive;
  //Encoders
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  //Odometry and Gyro
  private DifferentialDriveOdometry m_odometry;
  private Gyro m_gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
  //Feedforward and PID Controller
  private SimpleMotorFeedforward m_feedforward; 
  private PIDController leftPID;
  private PIDController rightPID;
  //Variables needed
  public Pose2d pose;
  public double speed;
  public double rotation;


  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    //Create the motors
    m_leftDriveBack = new CANSparkMax(leftDriveBackID, MotorType.kBrushless);
    m_leftDriveFront = new CANSparkMax(leftDriveFrontID, MotorType.kBrushless);
    m_rightDriveBack = new CANSparkMax(rightDriveBackID, MotorType.kBrushless);
    m_rightDriveFront = new CANSparkMax(rightDriveFrontID, MotorType.kBrushless);  
    m_leftDriveFront.setInverted(true);
    //Back follow front motors
    m_leftDriveBack.follow(m_leftDriveFront, false);
    m_rightDriveBack.follow(m_rightDriveFront);
    //DifferentialDrive
    m_drive = new DifferentialDrive(m_leftDriveFront, m_rightDriveFront);
    //Restore motor defaults
    m_leftDriveFront.restoreFactoryDefaults();
    m_rightDriveFront.restoreFactoryDefaults();
    m_leftDriveBack.restoreFactoryDefaults();
    m_rightDriveBack.restoreFactoryDefaults();
    //Encoders
    m_leftEncoder = m_leftDriveFront.getEncoder();
    m_rightEncoder = m_rightDriveFront.getEncoder();
    positionConversion(1 / 10.71 * 2 * Math.PI * Units.inchesToMeters(2));
    //Odomety
    m_odometry = new DifferentialDriveOdometry(getHeading());
    //Feedforward and PID
    m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts, 
    Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
    leftPID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    rightPID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    //Set Ramp Rate (Time till max speed)
    setRampRate();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block and updates the pose
    SmartDashboard.putNumber("leftEncoder", getLeftPosition());
    SmartDashboard.putNumber("rightEncoder", getRightPosition());
    pose = m_odometry.update(
      m_gyro.getRotation2d(), 
      m_leftEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(2) / 60, //speed of leftwheels in meters per second 
      m_rightEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(2) / 60 //speed of rightwheels in meters per second
      );
    m_drive.feed();
    SmartDashboard.updateValues();
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
  public void LeftDrive(double leftVelocitySetpoint) {
    m_leftDriveFront.setVoltage(m_feedforward.calculate(leftVelocitySetpoint)
      + leftPID.calculate(m_leftEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(2) / 60, 
      leftVelocitySetpoint));
  }

  public void RightDrive(double rightVelocitySetpoint){
    m_rightDriveFront.setVoltage(m_feedforward.calculate(rightVelocitySetpoint)
    + rightPID.calculate(m_rightEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(2) / 60,
    rightVelocitySetpoint)); 
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      m_leftEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(2) / 60, //speed of leftwheels in meters per second 
      m_rightEncoder.getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(2) / 60 //speed of rightwheels in meters per second
    );
  }

  public void setRampRate(){
    m_leftDriveBack.setClosedLoopRampRate(0.5);
    m_leftDriveFront.setClosedLoopRampRate(0.5);
    m_rightDriveBack.setClosedLoopRampRate(0.5);
    m_rightDriveFront.setClosedLoopRampRate(0.5);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void setOutput(double leftVolts, double rightVolts){
    m_leftDriveFront.set(leftVolts / 12);
    m_rightDriveFront.set(rightVolts / 12);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftPosition(){
    return m_leftEncoder.getPosition();
  }

  public double getRightPosition(){
    return m_rightEncoder.getPosition();
  }
    
  public void positionConversion(double convFactor){
    m_leftEncoder.setPositionConversionFactor(convFactor);
    m_rightEncoder.setPositionConversionFactor(convFactor);
  }

  public SimpleMotorFeedforward getFeedforward(){
    return m_feedforward;
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
    return pose;
  }
}
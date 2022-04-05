package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  private MotorControllerGroup leftGroup;
  private MotorControllerGroup rightGroup;
  //Differential Drive
  private DifferentialDrive m_drive;
  //Encoders
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  //Odometry and Gyro
  private DifferentialDriveOdometry m_odometry;
  private Gyro m_gyro = new ADXRS450_Gyro();
  //Feedforward and PID Controller
  private SimpleMotorFeedforward m_feedforward; 
  private PIDController leftPID;
  private PIDController rightPID;
  //Variables needed
  public Pose2d pose;


  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    //Initialize the Motors
    m_leftDriveBack = new CANSparkMax(leftDriveBackID, MotorType.kBrushless);
    m_leftDriveFront = new CANSparkMax(leftDriveFrontID, MotorType.kBrushless);
    m_rightDriveBack = new CANSparkMax(rightDriveBackID, MotorType.kBrushless);
    m_rightDriveFront = new CANSparkMax(rightDriveFrontID, MotorType.kBrushless);
    //Back follow front motors
    leftGroup = new MotorControllerGroup(m_leftDriveFront, m_leftDriveBack);
    rightGroup = new MotorControllerGroup(m_rightDriveFront, m_rightDriveBack);
    //DifferentialDrive
    leftGroup.setInverted(true);
    m_drive = new DifferentialDrive(leftGroup, rightGroup);
    setRampRate();
    setPeriodFrame();
    //Restore motor defaults
    m_leftDriveFront.restoreFactoryDefaults();
    m_rightDriveFront.restoreFactoryDefaults();
    m_leftDriveBack.restoreFactoryDefaults();
    m_rightDriveBack.restoreFactoryDefaults();

    m_leftDriveFront.setSmartCurrentLimit(90);
    m_leftDriveBack.setSmartCurrentLimit(90);
    m_rightDriveFront.setSmartCurrentLimit(90);
    m_rightDriveBack.setSmartCurrentLimit(90);

    m_leftDriveBack.setIdleMode(IdleMode.kBrake);
    m_leftDriveFront.setIdleMode(IdleMode.kBrake);
    m_rightDriveBack.setIdleMode(IdleMode.kBrake);
    m_rightDriveFront.setIdleMode(IdleMode.kBrake);

    //Disable safety features
    m_drive.setSafetyEnabled(false);
    //Encoders
    m_leftEncoder = m_leftDriveFront.getEncoder();
    m_rightEncoder = m_rightDriveFront.getEncoder();
    resetEncoders(); 
    //Odomety
    m_odometry = new DifferentialDriveOdometry(this.getGyroRotation());
    //Feedforward and PID
    m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts, 
    Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
    leftPID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    rightPID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  }

  public void tankDrive(double leftPower, double rightPower) {
    m_drive.tankDrive(leftPower, rightPower);
  }

  public void PIDDrive(double leftVelocitySetpoint, double rightVelocitySetpoint) {
    m_leftDriveFront.setVoltage(m_feedforward.calculate(leftVelocitySetpoint)
      + leftPID.calculate(m_leftEncoder.getVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(2) / 60, 
      leftVelocitySetpoint));
     m_rightDriveFront.setVoltage(m_feedforward.calculate(rightVelocitySetpoint)
    + rightPID.calculate(m_rightEncoder.getVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(2) / 60,
    rightVelocitySetpoint)); 
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      m_leftEncoder.getVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3) / 60, //speed of leftwheels in meters per second 
      m_rightEncoder.getVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3) / 60 //speed of rightwheels in meters per second
    );
  }

  public void setRampRate(){
    m_leftDriveBack.setOpenLoopRampRate(1);
    m_rightDriveBack.setOpenLoopRampRate(1);
    m_leftDriveFront.setOpenLoopRampRate(1);
    m_rightDriveFront.setOpenLoopRampRate(1);
  }

  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void setOutput(double leftVolts, double rightVolts){
    m_leftDriveFront.set(leftVolts / 12);
    m_rightDriveFront.set(rightVolts / 12);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, this.getGyroRotation());
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

  public void setPeriodFrame(){
    m_leftDriveFront.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_leftDriveBack.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_rightDriveFront.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_rightDriveBack.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_leftDriveFront.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_leftDriveBack.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_rightDriveFront.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_rightDriveBack.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block and updates the pose
    double leftBackTemp = m_leftDriveBack.getMotorTemperature();
    double leftFrontTemp = m_leftDriveFront.getMotorTemperature();
    double rightBackTemp = m_rightDriveBack.getMotorTemperature();
    double rightFrontTemp = m_rightDriveFront.getMotorTemperature();

    SmartDashboard.putNumber("Left Front Temp", leftFrontTemp);
    SmartDashboard.putNumber("Left Back Temp", leftBackTemp);
    SmartDashboard.putNumber("Right Back Temp", rightBackTemp);
    SmartDashboard.putNumber("Right Front Temp", rightFrontTemp);
    
    SmartDashboard.putNumber("leftEncoder", getLeftPosition());
    SmartDashboard.putNumber("rightEncoder", -getRightPosition());
    pose = m_odometry.update(
      this.getGyroRotation(), 
      m_leftEncoder.getVelocity() / 5.95 * 2 * Math.PI * Units.inchesToMeters(3) / 60, //speed of leftwheels in meters per second 
      m_rightEncoder.getVelocity() / 5.95 * 2 * Math.PI * Units.inchesToMeters(3) / 60 //speed of rightwheels in meters per second
    );
  }
}
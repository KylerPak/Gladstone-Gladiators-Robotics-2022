// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //motors
    public static int leftDriveFrontCANID = 1;
    public static int leftDriveBackCANID = 2;
    public static int rightDriveFrontCANID = 3;
    public static int rightDriveBackCANID = 4;
    public static int ShooterCANID = 5;
    public static int IntakeCANID = 6;
    public static int FeedMotorCANID = 7;

    //Encoder Values
    public static final double ksVolts = 0.22; //subject to change
    public static final double kvVoltSecondsPerMeter = 1.98; //subject to change
    public static final double kaVoltSecondsSquaredPerMeter = 0.2; //subject to change
    public static final double kPDriveVel = 8.5; //subject to change

    //DifferentialDriveKinematics
    public static final double kTrackwidthMeters = 0.69; //subject to change
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    //MaxSpeed and Accelleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    //Ramsette Perameters
    public static final double kRamseteB = 2; //subject to change
    public static final double kRamseteZeta = 0.7; //subject to change
}

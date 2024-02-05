// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveDriveConstants {
    public static final double kP = 0.0046;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double rKP = 0.03;
    public static final double rKI = 0.0;
    public static final double rKD = 0.0;
    public static final double frontLeftSensor = 257.896;
    public static final double frontRightSensor = 50.37;
    public static final double backLeftSensor = 17.4;
    public static final double backRightSensor = 115.899;
    public static final double robotSideLength = 24.25;//INCH
    public static final double robotWheelDiameter = 4;//INCH
  }
}

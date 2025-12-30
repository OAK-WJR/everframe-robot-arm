// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ArmJointConstants {
    public static final int CAN_ID = 2;
    public static final double GEAR_RATIO = 100.0; // motor:joint
    public static final double Ks = 0.2095, Kv = 3.0, Ka = 0; // Kv = 12.087, Ka = 0.81667
    public static final double kP = 35.0, kI = 100.0, kD = 1.2;
    public static final double MAX_VEL = 6.0, MAX_ACC = 12.0; // rad/s, rad/s^2
    public static final double DT = 0.02;
    public static final double NOMINAL_VOLTAGE = 12.0;
  }
}

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
  public static final class ArmConstants {
    public static final double DT = 0.02;
    public static final double NOMINAL_VOLTAGE = 12.0;

    public enum RevController {
      FLEX,
      MAX
    }

    /** Defaults published to NetworkTables for each joint (can be tuned at runtime). */
    public static final class JointDefaults {
      public static final double KS = 0.2095, KV = 3.0, KA = 0.0; // (historical) KV=12.087, KA=0.81667
      /** Gravity feedforward for arm-like joints (V). Used as kG * cos(thetaRad). */
      public static final double KG = 0.0;
      public static final double KP = 35.0, KI = 100.0, KD = 1.2;
      public static final double MAX_VOLTS = 5.0;
      public static final double IZONE_DEG = 0.5;
      public static final double I_MAX_VOLTS = 1.0;
      public static final double MAX_VEL = 6.0, MAX_ACC = 12.0; // rad/s, rad/s^2
    }

    /** J1 defaults (can differ from J2). */
    public static final class J1Defaults {
      public static final double KS = JointDefaults.KS, KV = JointDefaults.KV, KA = JointDefaults.KA;
      public static final double KG = JointDefaults.KG;
      public static final double KP = JointDefaults.KP, KI = JointDefaults.KI, KD = JointDefaults.KD;
      public static final double MAX_VOLTS = JointDefaults.MAX_VOLTS;
      public static final double IZONE_DEG = JointDefaults.IZONE_DEG;
      public static final double I_MAX_VOLTS = JointDefaults.I_MAX_VOLTS;
      public static final double MAX_VEL = JointDefaults.MAX_VEL, MAX_ACC = JointDefaults.MAX_ACC;
    }

    /** J2 defaults (can differ from J1). */
    public static final class J2Defaults {
      // TODO: 如需不同默认值，在这里改
      public static final double KS = JointDefaults.KS, KV = JointDefaults.KV, KA = JointDefaults.KA;
      public static final double KG = JointDefaults.KG;
      public static final double KP = JointDefaults.KP, KI = JointDefaults.KI, KD = JointDefaults.KD;
      public static final double MAX_VOLTS = JointDefaults.MAX_VOLTS;
      public static final double IZONE_DEG = JointDefaults.IZONE_DEG;
      public static final double I_MAX_VOLTS = JointDefaults.I_MAX_VOLTS;
      public static final double MAX_VEL = JointDefaults.MAX_VEL, MAX_ACC = JointDefaults.MAX_ACC;
    }

    /** J3 defaults (can differ). */
    public static final class J3Defaults {
      // TODO: 如需不同默认值，在这里改（Spark MAX + NEO v1.1）
      public static final double KS = 0.19, KV = JointDefaults.KV, KA = JointDefaults.KA;
      public static final double KG = -0.05;
      // Synced from web tuning
      public static final double KP = 40.0, KI = 100.0, KD = 0.7;
      public static final double MAX_VOLTS = JointDefaults.MAX_VOLTS;
      public static final double IZONE_DEG = 0.2;
      public static final double I_MAX_VOLTS = 1.0;
      public static final double MAX_VEL = JointDefaults.MAX_VEL, MAX_ACC = JointDefaults.MAX_ACC;
    }

    public static final class J1 {
      public static final String NAME = "J1";
      public static final int CAN_ID = 2;
      public static final double GEAR_RATIO = 100.0; // motor:joint
      public static final double MIN_DEG = -180.0;
      public static final double MAX_DEG = 180.0;
      public static final RevController CONTROLLER = RevController.FLEX;
    }

    // TODO: 修改为 J2 实际参数
    public static final class J2 {
      public static final String NAME = "J2";
      public static final int CAN_ID = 3;
      public static final double GEAR_RATIO = 100.0; // motor:joint
      public static final double MIN_DEG = -210.0;
      public static final double MAX_DEG = 30.0;
      public static final RevController CONTROLLER = RevController.FLEX;
    }

    // TODO: 修改为 J3 实际限位
    public static final class J3 {
      public static final String NAME = "J3";
      public static final int CAN_ID = 4;
      public static final double GEAR_RATIO = 100.0; // motor:joint
      public static final double MIN_DEG = -150.0;
      public static final double MAX_DEG = 150.0;
      /** 若硬件未接好会导致 CAN 超时拖慢循环，可先关掉。 */
      public static final boolean ENABLED = true;
      /** 你日志提示 “not a SPARK MAX”，如果确实是 Spark Flex 请改为 FLEX。 */
      public static final RevController CONTROLLER = RevController.MAX;
    }
  }
}

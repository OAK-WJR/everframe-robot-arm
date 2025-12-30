package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/** Arm subsystem that owns multiple joints (J1/J2/J3). */
public class ArmSubsystem extends SubsystemBase {
  private final NetworkTableEntry activeJoint =
      NetworkTableInstance.getDefault().getTable("Arm").getEntry("ActiveJoint"); // "J1" / "J2" / "J3" / "Both"

  private final TunableJoint j1 =
      new TunableJoint(
          ArmConstants.J1.NAME,
          ArmConstants.J1.CONTROLLER == ArmConstants.RevController.MAX
              ? TunableJoint.ControllerType.MAX
              : TunableJoint.ControllerType.FLEX,
          ArmConstants.J1.CAN_ID,
          ArmConstants.J1.GEAR_RATIO,
          ArmConstants.J1.MIN_DEG,
          ArmConstants.J1.MAX_DEG,
          new TunableJoint.Defaults(
              ArmConstants.J1Defaults.KS,
              ArmConstants.J1Defaults.KV,
              ArmConstants.J1Defaults.KA,
              ArmConstants.J1Defaults.KG,
              ArmConstants.J1Defaults.KP,
              ArmConstants.J1Defaults.KI,
              ArmConstants.J1Defaults.KD,
              ArmConstants.J1Defaults.MAX_VOLTS,
              ArmConstants.J1Defaults.IZONE_DEG,
              ArmConstants.J1Defaults.I_MAX_VOLTS,
              ArmConstants.J1Defaults.MAX_VEL,
              ArmConstants.J1Defaults.MAX_ACC),
          NetworkTableInstance.getDefault().getTable("Arm").getSubTable(ArmConstants.J1.NAME));

  private final TunableJoint j2 =
      new TunableJoint(
          ArmConstants.J2.NAME,
          ArmConstants.J2.CONTROLLER == ArmConstants.RevController.MAX
              ? TunableJoint.ControllerType.MAX
              : TunableJoint.ControllerType.FLEX,
          ArmConstants.J2.CAN_ID,
          ArmConstants.J2.GEAR_RATIO,
          ArmConstants.J2.MIN_DEG,
          ArmConstants.J2.MAX_DEG,
          new TunableJoint.Defaults(
              ArmConstants.J2Defaults.KS,
              ArmConstants.J2Defaults.KV,
              ArmConstants.J2Defaults.KA,
              ArmConstants.J2Defaults.KG,
              ArmConstants.J2Defaults.KP,
              ArmConstants.J2Defaults.KI,
              ArmConstants.J2Defaults.KD,
              ArmConstants.J2Defaults.MAX_VOLTS,
              ArmConstants.J2Defaults.IZONE_DEG,
              ArmConstants.J2Defaults.I_MAX_VOLTS,
              ArmConstants.J2Defaults.MAX_VEL,
              ArmConstants.J2Defaults.MAX_ACC),
          NetworkTableInstance.getDefault().getTable("Arm").getSubTable(ArmConstants.J2.NAME));

  private final TunableJoint j3 =
      ArmConstants.J3.ENABLED
          ? new TunableJoint(
              ArmConstants.J3.NAME,
              ArmConstants.J3.CONTROLLER == ArmConstants.RevController.MAX
                  ? TunableJoint.ControllerType.MAX
                  : TunableJoint.ControllerType.FLEX,
              ArmConstants.J3.CAN_ID,
              ArmConstants.J3.GEAR_RATIO,
              ArmConstants.J3.MIN_DEG,
              ArmConstants.J3.MAX_DEG,
              new TunableJoint.Defaults(
                  ArmConstants.J3Defaults.KS,
                  ArmConstants.J3Defaults.KV,
                  ArmConstants.J3Defaults.KA,
                  ArmConstants.J3Defaults.KG,
                  ArmConstants.J3Defaults.KP,
                  ArmConstants.J3Defaults.KI,
                  ArmConstants.J3Defaults.KD,
                  ArmConstants.J3Defaults.MAX_VOLTS,
                  ArmConstants.J3Defaults.IZONE_DEG,
                  ArmConstants.J3Defaults.I_MAX_VOLTS,
                  ArmConstants.J3Defaults.MAX_VEL,
                  ArmConstants.J3Defaults.MAX_ACC),
              NetworkTableInstance.getDefault().getTable("Arm").getSubTable(ArmConstants.J3.NAME))
          : null;

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    String which = activeJoint.getString("");
    if (which == null || which.isEmpty()) {
      which = ArmConstants.J1.NAME;
      activeJoint.setString(which);
    }
    if (which.equalsIgnoreCase(ArmConstants.J1.NAME)) {
      j1.update(now);
      j2.inactive();
      if (j3 != null) j3.inactive();
    } else if (which.equalsIgnoreCase(ArmConstants.J2.NAME)) {
      j2.update(now);
      j1.inactive();
      if (j3 != null) j3.inactive();
    } else if (which.equalsIgnoreCase(ArmConstants.J3.NAME)) {
      if (j3 != null) j3.update(now);
      j1.inactive();
      j2.inactive();
    } else {
      // "Both" (or anything else): keep old behavior
      j1.update(now);
      j2.update(now);
      if (j3 != null) j3.update(now);
    }
  }
}



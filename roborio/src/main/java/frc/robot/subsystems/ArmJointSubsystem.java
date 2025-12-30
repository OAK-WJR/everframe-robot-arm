package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmJointConstants;

public class ArmJointSubsystem extends SubsystemBase {
  private final SparkFlex motor = new SparkFlex(ArmJointConstants.CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder enc = motor.getEncoder(); // rad / rad/s after config
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          ArmJointConstants.kP,
          ArmJointConstants.kI,
          ArmJointConstants.kD,
          new TrapezoidProfile.Constraints(ArmJointConstants.MAX_VEL, ArmJointConstants.MAX_ACC));
  private double prevSpVel = 0.0;
  private double lastOutVolts = 0.0;

  // Telemetry for web dashboard (NT keys under "ArmJoint")
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("ArmJoint");
  // Tunables (written by web UI)
  private final NetworkTableEntry ntTuneLive = nt.getEntry("TuneLive");
  private final NetworkTableEntry ntkP = nt.getEntry("kP");
  private final NetworkTableEntry ntkI = nt.getEntry("kI");
  private final NetworkTableEntry ntkD = nt.getEntry("kD");
  private final NetworkTableEntry ntKs = nt.getEntry("Ks");
  private final NetworkTableEntry ntKv = nt.getEntry("Kv");
  private final NetworkTableEntry ntKa = nt.getEntry("Ka");
  private final NetworkTableEntry ntMaxVolts = nt.getEntry("MaxVolts");
  // --- Anti-windup (必须标注) ---
  // IZoneDeg: only allow integral accumulation when |position error| is within this zone.
  // IMaxVolts: clamp integral term contribution to output to +/- this value.
  private final NetworkTableEntry ntIZoneDeg = nt.getEntry("IZoneDeg");
  private final NetworkTableEntry ntIMaxVolts = nt.getEntry("IMaxVolts");

  private double curkP = ArmJointConstants.kP, curkI = ArmJointConstants.kI, curkD = ArmJointConstants.kD;
  private double curKs = ArmJointConstants.Ks, curKv = ArmJointConstants.Kv, curKa = ArmJointConstants.Ka;
  private double curMaxVolts = 12.0;
  // --- Anti-windup (必须标注) defaults ---
  private double curIZoneDeg = 0.5;
  private double curIMaxVolts = 1.0;

  private final NetworkTableEntry ntAngleDeg = nt.getEntry("AngleDeg");
  private final NetworkTableEntry ntGoalDeg = nt.getEntry("GoalDeg");
  private final NetworkTableEntry ntOutVolts = nt.getEntry("OutVoltsCmd");
  private final NetworkTableEntry ntEnabled = nt.getEntry("Enabled");

  public ArmJointSubsystem() {
    var cfg = new SparkFlexConfig()
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(ArmJointConstants.NOMINAL_VOLTAGE);
    cfg.encoder.positionConversionFactor(2.0 * Math.PI / ArmJointConstants.GEAR_RATIO);
    cfg.encoder.velocityConversionFactor(
        2.0 * Math.PI / ArmJointConstants.GEAR_RATIO / 60.0);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pid.setTolerance(Math.toRadians(1.0));
    pid.reset(getAngleRad());

    // Publish defaults for external dashboard to edit
    ntTuneLive.setBoolean(false);
    ntkP.setDouble(curkP);
    ntkI.setDouble(curkI);
    ntkD.setDouble(curkD);
    ntKs.setDouble(curKs);
    ntKv.setDouble(curKv);
    ntKa.setDouble(curKa);
    ntMaxVolts.setDouble(curMaxVolts);
    // --- Anti-windup (必须标注) publish defaults ---
    ntIZoneDeg.setDouble(curIZoneDeg);
    ntIMaxVolts.setDouble(curIMaxVolts);

    // Apply defaults immediately
    pid.setIntegratorRange(-curIMaxVolts, curIMaxVolts);
    pid.setIZone(Math.toRadians(curIZoneDeg));

    ntAngleDeg.setDouble(0.0);
    ntGoalDeg.setDouble(0.0);
    ntOutVolts.setDouble(0.0);
    ntEnabled.setBoolean(false);
  }

  public double getAngleRad() { return enc.getPosition(); }
  public double getAngleDeg() { return Math.toDegrees(getAngleRad()); }
  public void setGoalDeg(double deg) { pid.setGoal(Math.toRadians(deg)); }
  public void zero() { enc.setPosition(0.0); pid.reset(0.0); prevSpVel = 0.0; }

  @Override
  public void periodic() {
    boolean enabled = DriverStation.isEnabled();

    // Update tunables:
    // - If TuneLive=true, apply while enabled.
    // - Otherwise, only apply while disabled (safer).
    boolean tuneLive = ntTuneLive.getBoolean(false);
    if (!enabled || tuneLive) {
      double newkP = ntkP.getDouble(curkP);
      double newkI = ntkI.getDouble(curkI);
      double newkD = ntkD.getDouble(curkD);
      if (newkP != curkP || newkI != curkI || newkD != curkD) {
        curkP = newkP; curkI = newkI; curkD = newkD;
        pid.setPID(curkP, curkI, curkD);
      }
      curKs = ntKs.getDouble(curKs);
      curKv = ntKv.getDouble(curKv);
      curKa = ntKa.getDouble(curKa);
    }
    // MaxVolts is always safe to apply
    curMaxVolts = MathUtil.clamp(ntMaxVolts.getDouble(curMaxVolts), 0.0, 12.0);
    // --- Anti-windup (必须标注) always safe to apply ---
    // If IZoneDeg <= 0, disable IZone behavior by setting it to +inf (avoid exceptions for 0/negative).
    curIZoneDeg = ntIZoneDeg.getDouble(curIZoneDeg);
    if (curIZoneDeg <= 0.0) {
      pid.setIZone(Double.POSITIVE_INFINITY);
    } else {
      pid.setIZone(Math.toRadians(curIZoneDeg));
    }
    curIMaxVolts = MathUtil.clamp(ntIMaxVolts.getDouble(curIMaxVolts), 0.0, 12.0);
    pid.setIntegratorRange(-curIMaxVolts, curIMaxVolts);

    ntEnabled.setBoolean(enabled);
    ntAngleDeg.setDouble(getAngleDeg());
    ntGoalDeg.setDouble(Math.toDegrees(pid.getGoal().position));
    ntOutVolts.setDouble(lastOutVolts);

    if (!enabled) { motor.setVoltage(0.0); lastOutVolts = 0.0; return; }
    var sp = pid.getSetpoint();
    double accel = (sp.velocity - prevSpVel) / ArmJointConstants.DT;
    prevSpVel = sp.velocity;
    // Ks direction: follow desired velocity direction; when |v_sp| is ~0 (startup / stop),
    // fall back to the *planned* direction (goalPos - setpointPos), NOT measurement error direction.
    double vel = sp.velocity;
    double plannedDir = Math.signum(pid.getGoal().position - sp.position);
    double sign = Math.abs(vel) > 1e-4 ? Math.signum(vel) : plannedDir;
    double ffVolts =
        curKs * sign
            + curKv * vel
            // Ka 不做反向（避免减速段“回拉”过猛）：只沿 sign 方向提供加速度前馈
            + curKa * Math.abs(accel) * sign;
    double volts = pid.calculate(getAngleRad()) + ffVolts;
    lastOutVolts = MathUtil.clamp(volts, -curMaxVolts, curMaxVolts);
    motor.setVoltage(lastOutVolts);
  }
}



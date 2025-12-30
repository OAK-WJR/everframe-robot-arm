package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.SineGoalGenerator;

/** Single-joint controller with NetworkTables tunables + telemetry. */
public final class TunableJoint {
  public enum ControllerType {
    FLEX,
    MAX
  }

  public static final class Defaults {
    public final double ks, kv, ka, kg;
    public final double kp, ki, kd;
    public final double maxVolts;
    public final double iZoneDeg;
    public final double iMaxVolts;
    public final double maxVel;
    public final double maxAcc;

    public Defaults(
        double ks,
        double kv,
        double ka,
        double kg,
        double kp,
        double ki,
        double kd,
        double maxVolts,
        double iZoneDeg,
        double iMaxVolts,
        double maxVel,
        double maxAcc) {
      this.ks = ks;
      this.kv = kv;
      this.ka = ka;
      this.kg = kg;
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
      this.maxVolts = maxVolts;
      this.iZoneDeg = iZoneDeg;
      this.iMaxVolts = iMaxVolts;
      this.maxVel = maxVel;
      this.maxAcc = maxAcc;
    }
  }

  private final String name;
  private final SparkBase motor;
  private final RelativeEncoder enc;
  private final ProfiledPIDController pid;
  private final double baseMinDeg;
  private final double baseMaxDeg;

  private double prevSpVel = 0.0;
  private double lastOutVolts = 0.0;
  private double lastDir = 0.0;
  private final double accelLimitRadPerS2;

  // Control inputs
  private final NetworkTableEntry targetDeg;
  private final NetworkTableEntry zero;
  private final NetworkTableEntry testEnable;
  private final NetworkTableEntry testAmpDeg;
  private final NetworkTableEntry testPeriodS;
  private final NetworkTableEntry testOffsetDeg;

  // Tunables
  private final NetworkTableEntry tuneLive;
  private final NetworkTableEntry ntkP, ntkI, ntkD;
  private final NetworkTableEntry ntKs, ntKv, ntKa, ntKg;
  private final NetworkTableEntry ntMaxVolts;
  // Soft limits (degrees)
  private final NetworkTableEntry ntMinDeg;
  private final NetworkTableEntry ntMaxDeg;
  // --- Anti-windup (必须标注) ---
  private final NetworkTableEntry ntIZoneDeg;
  private final NetworkTableEntry ntIMaxVolts;

  // Telemetry
  private final NetworkTableEntry ntAngleDeg, ntGoalDeg, ntOutVoltsCmd, ntEnabled;
  private final NetworkTableEntry ntInputTargetDeg, ntGoalSource;
  private final NetworkTableEntry ntActive;

  // Current applied values
  private double curkP, curkI, curkD;
  private double curKs, curKv, curKa, curKg;
  private double curMaxVolts;
  private double curMinDeg, curMaxDeg;
  private double curIZoneDeg, curIMaxVolts;

  // Sine test phase continuity
  private boolean prevTestEnable = false;
  private double testStartTime = 0.0;
  private double prevTestAmp = 0.0, prevTestPeriod = 0.0, prevTestOffset = 0.0;

  public TunableJoint(
      String name,
      ControllerType controllerType,
      int canId,
      double gearRatio,
      double minDeg,
      double maxDeg,
      Defaults defaults,
      NetworkTable nt) {
    this.name = name;
    this.baseMinDeg = minDeg;
    this.baseMaxDeg = maxDeg;
    this.accelLimitRadPerS2 = Math.max(0.0, defaults.maxAcc);
    motor =
        controllerType == ControllerType.MAX
            ? new SparkMax(canId, MotorType.kBrushless)
            : new SparkFlex(canId, MotorType.kBrushless);
    enc = motor.getEncoder();

    var cfg =
        controllerType == ControllerType.MAX
            ? new SparkMaxConfig().idleMode(IdleMode.kBrake).voltageCompensation(ArmConstants.NOMINAL_VOLTAGE)
            : new SparkFlexConfig().idleMode(IdleMode.kBrake).voltageCompensation(ArmConstants.NOMINAL_VOLTAGE);
    cfg.encoder.positionConversionFactor(2.0 * Math.PI / gearRatio);
    cfg.encoder.velocityConversionFactor(2.0 * Math.PI / gearRatio / 60.0);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pid =
        new ProfiledPIDController(
            defaults.kp,
            defaults.ki,
            defaults.kd,
            new TrapezoidProfile.Constraints(defaults.maxVel, defaults.maxAcc),
            ArmConstants.DT);
    pid.setTolerance(Math.toRadians(1.0));
    pid.reset(getAngleRad());

    // Entries
    targetDeg = nt.getEntry("TargetDeg");
    zero = nt.getEntry("Zero");
    testEnable = nt.getEntry("TestEnable");
    testAmpDeg = nt.getEntry("TestAmpDeg");
    testPeriodS = nt.getEntry("TestPeriodS");
    testOffsetDeg = nt.getEntry("TestOffsetDeg");

    tuneLive = nt.getEntry("TuneLive");
    ntkP = nt.getEntry("kP");
    ntkI = nt.getEntry("kI");
    ntkD = nt.getEntry("kD");
    ntKs = nt.getEntry("Ks");
    ntKv = nt.getEntry("Kv");
    ntKa = nt.getEntry("Ka");
    ntKg = nt.getEntry("kG");
    ntMaxVolts = nt.getEntry("MaxVolts");
    ntMinDeg = nt.getEntry("MinDeg");
    ntMaxDeg = nt.getEntry("MaxDeg");
    ntIZoneDeg = nt.getEntry("IZoneDeg");
    ntIMaxVolts = nt.getEntry("IMaxVolts");

    ntAngleDeg = nt.getEntry("AngleDeg");
    ntGoalDeg = nt.getEntry("GoalDeg");
    ntOutVoltsCmd = nt.getEntry("OutVoltsCmd");
    ntEnabled = nt.getEntry("Enabled");
    ntInputTargetDeg = nt.getEntry("InputTargetDeg");
    ntGoalSource = nt.getEntry("GoalSource"); // "manual" or "sine"
    ntActive = nt.getEntry("Active");

    // Defaults (publish once)
    targetDeg.setDouble(0.0);
    zero.setBoolean(false);
    testEnable.setBoolean(false);
    testAmpDeg.setDouble(30.0);
    testPeriodS.setDouble(4.0);
    testOffsetDeg.setDouble(0.0);

    tuneLive.setBoolean(false);

    curkP = defaults.kp;
    curkI = defaults.ki;
    curkD = defaults.kd;
    curKs = defaults.ks;
    curKv = defaults.kv;
    curKa = defaults.ka;
    curKg = defaults.kg;
    curMaxVolts = defaults.maxVolts;
    curIZoneDeg = defaults.iZoneDeg;
    curIMaxVolts = defaults.iMaxVolts;
    curMinDeg = minDeg;
    curMaxDeg = maxDeg;

    ntkP.setDouble(curkP);
    ntkI.setDouble(curkI);
    ntkD.setDouble(curkD);
    ntKs.setDouble(curKs);
    ntKv.setDouble(curKv);
    ntKa.setDouble(curKa);
    ntKg.setDouble(curKg);
    ntMaxVolts.setDouble(curMaxVolts);
    ntMinDeg.setDouble(curMinDeg);
    ntMaxDeg.setDouble(curMaxDeg);
    ntIZoneDeg.setDouble(curIZoneDeg);
    ntIMaxVolts.setDouble(curIMaxVolts);

    // Apply defaults immediately
    pid.setIntegratorRange(-curIMaxVolts, curIMaxVolts);
    pid.setIZone(Math.toRadians(curIZoneDeg));

    ntAngleDeg.setDouble(0.0);
    ntGoalDeg.setDouble(0.0);
    ntOutVoltsCmd.setDouble(0.0);
    ntEnabled.setBoolean(false);
    ntInputTargetDeg.setDouble(0.0);
    ntGoalSource.setString("manual");
    ntActive.setBoolean(true);
  }

  public String getName() { return name; }
  public double getAngleRad() { return enc.getPosition(); }
  public double getAngleDeg() { return Math.toDegrees(getAngleRad()); }

  public void zeroNow() {
    enc.setPosition(0.0);
    pid.reset(0.0);
    prevSpVel = 0.0;
    lastDir = 0.0;
  }

  /** When not the selected joint: hold current angle and output 0V. */
  public void inactive() {
    pid.reset(getAngleRad());
    motor.setVoltage(0.0);
    lastOutVolts = 0.0;
    ntActive.setBoolean(false);
    ntEnabled.setBoolean(DriverStation.isEnabled());
    ntAngleDeg.setDouble(getAngleDeg());
    ntOutVoltsCmd.setDouble(0.0);
  }

  /** Called by ArmSubsystem every 20ms. */
  public void update(double nowSec) {
    boolean enabled = DriverStation.isEnabled();
    ntActive.setBoolean(true);

    // One-shot zero
    if (zero.getBoolean(false)) {
      zeroNow();
      zero.setBoolean(false);
    }

    // Apply tunables
    boolean live = tuneLive.getBoolean(false);
    if (!enabled || live) {
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
      curKg = ntKg.getDouble(curKg);
    }
    curMaxVolts = MathUtil.clamp(ntMaxVolts.getDouble(curMaxVolts), 0.0, 12.0);
    // Soft limits are always safe to apply
    curMinDeg = ntMinDeg.getDouble(curMinDeg);
    curMaxDeg = ntMaxDeg.getDouble(curMaxDeg);
    if (curMaxDeg < curMinDeg) {
      double tmp = curMinDeg;
      curMinDeg = curMaxDeg;
      curMaxDeg = tmp;
    }
    // If limits are invalid (e.g., both 0 due to stale UI state), fall back to defaults.
    if (Math.abs(curMaxDeg - curMinDeg) < 1e-3) {
      curMinDeg = baseMinDeg;
      curMaxDeg = baseMaxDeg;
      ntMinDeg.setDouble(curMinDeg);
      ntMaxDeg.setDouble(curMaxDeg);
    }

    // --- Anti-windup (必须标注) always safe to apply ---
    curIZoneDeg = ntIZoneDeg.getDouble(curIZoneDeg);
    if (curIZoneDeg <= 0.0) {
      pid.setIZone(Double.POSITIVE_INFINITY);
    } else {
      pid.setIZone(Math.toRadians(curIZoneDeg));
    }
    curIMaxVolts = MathUtil.clamp(ntIMaxVolts.getDouble(curIMaxVolts), 0.0, 12.0);
    pid.setIntegratorRange(-curIMaxVolts, curIMaxVolts);

    // Goal source: sine test or manual target
    boolean test = testEnable.getBoolean(false);
    double goalDeg;
    if (test) {
      double amp = testAmpDeg.getDouble(30.0);
      double period = testPeriodS.getDouble(4.0);
      double offset = testOffsetDeg.getDouble(0.0);
      if (!prevTestEnable) {
        // Align phase so the sine starts at the current angle (reduces step -> reduces voltage spike).
        pid.reset(getAngleRad());
        if (period > 0.0 && Math.abs(amp) > 1e-6) {
          double desired = MathUtil.clamp(getAngleDeg(), curMinDeg, curMaxDeg);
          double ratio = (desired - offset) / amp;
          ratio = MathUtil.clamp(ratio, -1.0, 1.0);
          double phi = Math.asin(ratio); // sin(phi)=ratio
          double t0 = (phi * period) / (2.0 * Math.PI);
          testStartTime = nowSec - t0;
        } else {
          testStartTime = nowSec;
        }
        prevTestAmp = amp; prevTestPeriod = period; prevTestOffset = offset;
      } else {
        boolean changed = amp != prevTestAmp || period != prevTestPeriod || offset != prevTestOffset;
        if (changed && prevTestPeriod > 0.0 && period > 0.0) {
          double oldPhase = 2.0 * Math.PI * (nowSec - testStartTime) / prevTestPeriod;
          testStartTime = nowSec - (oldPhase * period / (2.0 * Math.PI));
          prevTestAmp = amp; prevTestPeriod = period; prevTestOffset = offset;
        }
      }
      double t = nowSec - testStartTime;
      goalDeg = SineGoalGenerator.goalDeg(t, offset, amp, period);
      ntGoalSource.setString("sine");
    } else {
      goalDeg = targetDeg.getDouble(0.0);
      ntGoalSource.setString("manual");
    }
    prevTestEnable = test;
    // Apply joint angle limits
    goalDeg = MathUtil.clamp(goalDeg, curMinDeg, curMaxDeg);
    pid.setGoal(Math.toRadians(goalDeg));

    // Telemetry (inputs)
    ntEnabled.setBoolean(enabled);
    ntAngleDeg.setDouble(getAngleDeg());
    ntGoalDeg.setDouble(goalDeg);
    ntInputTargetDeg.setDouble(targetDeg.getDouble(0.0));

    if (!enabled) {
      motor.setVoltage(0.0);
      lastOutVolts = 0.0;
      ntOutVoltsCmd.setDouble(0.0);
      return;
    }

    // Control
    var sp = pid.getSetpoint();
    double accel = (sp.velocity - prevSpVel) / ArmConstants.DT;
    if (accelLimitRadPerS2 > 0.0) {
      // Finite-difference accel is noisy; clamp to something sane (profile constraint).
      accel = MathUtil.clamp(accel, -accelLimitRadPerS2, accelLimitRadPerS2);
    }
    prevSpVel = sp.velocity;
    double vel = sp.velocity;
    // Robust direction for Ks/Ka near stop:
    // - Prefer desired velocity sign when moving
    // - Otherwise use goal - setpoint position, but with a small deadband to avoid sign flapping
    // - Otherwise hold last non-zero direction (prevents +/-Ks jitter around 0)
    double posErr = pid.getGoal().position - sp.position;
    double sign;
    if (Math.abs(vel) > 1e-4) {
      sign = Math.signum(vel);
    } else if (Math.abs(posErr) > Math.toRadians(0.25)) {
      sign = Math.signum(posErr);
    } else {
      sign = lastDir;
    }
    if (sign != 0.0) lastDir = sign;

    // Gravity feedforward for arm-like joints.
    // Uses setpoint angle (rad). Ensure your "zero" angle matches the physical reference you want.
    double gVolts = curKg * Math.cos(sp.position);

    double ffVolts =
        curKs * sign
            + gVolts
            + curKv * vel
            // Ka 不做反向：只沿 sign 方向提供加速度前馈
            + curKa * Math.abs(accel) * sign;
    double volts = pid.calculate(getAngleRad()) + ffVolts;
    lastOutVolts = MathUtil.clamp(volts, -curMaxVolts, curMaxVolts);
    motor.setVoltage(lastOutVolts);
    ntOutVoltsCmd.setDouble(lastOutVolts);
  }
}



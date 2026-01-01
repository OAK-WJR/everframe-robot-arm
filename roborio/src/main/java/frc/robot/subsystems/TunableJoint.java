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
    public final double kg2, kg2OtherScale;
    public final double globalOtherScale;
    public final int goalMode;
    public final double goalOtherScale;
    public final boolean useGlobalAngleForGravity;
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
        double kg2,
        double kg2OtherScale,
        double globalOtherScale,
        int goalMode,
        double goalOtherScale,
        boolean useGlobalAngleForGravity,
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
      this.kg2 = kg2;
      this.kg2OtherScale = kg2OtherScale;
      this.globalOtherScale = globalOtherScale;
      this.goalMode = goalMode;
      this.goalOtherScale = goalOtherScale;
      this.useGlobalAngleForGravity = useGlobalAngleForGravity;
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
  private final double ioSign; // +1 normal, -1 inverted (motor + encoder)
  private final Defaults defaultsCfg;

  private double prevSpVel = 0.0;
  private double lastOutVolts = 0.0;
  private double lastDir = 0.0;
  private final double accelLimitRadPerS2;

  // Control inputs
  private final NetworkTableEntry targetDeg;
  private final NetworkTableEntry zero;
  private final NetworkTableEntry setPosDeg;
  private final NetworkTableEntry setPos;
  private final NetworkTableEntry resetDefaults;
  private final NetworkTableEntry testEnable;
  private final NetworkTableEntry testAmpDeg;
  private final NetworkTableEntry testPeriodS;
  private final NetworkTableEntry testOffsetDeg;

  // Tunables
  private final NetworkTableEntry tuneLive;
  private final NetworkTableEntry ntkP, ntkI, ntkD;
  private final NetworkTableEntry ntKs, ntKv, ntKa, ntKg;
  // Coupling
  private final NetworkTableEntry ntGoalRelativeToOther; // if true: GoalDeg += OtherAngleDeg
  private final NetworkTableEntry ntkG2; // extra gravity term coefficient (V)
  private final NetworkTableEntry ntkG2OtherScale; // g2 = kG2*cos(self + other*scale)
  private final NetworkTableEntry ntGlobalOtherScale; // global = self + scale*other
  private final NetworkTableEntry ntGoalMode; // 0=local, 1=follow/add other, 2=global (local=goal-other)
  private final NetworkTableEntry ntGoalOtherScale;
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
  private final NetworkTableEntry ntOtherAngleDeg, ntAngleDegGlobal, ntGoalDegGlobal;

  // Current applied values
  private double curkP, curkI, curkD;
  private double curKs, curKv, curKa, curKg;
  private boolean curGoalRelativeToOther;
  private double curkG2, curkG2OtherScale;
  private double curGlobalOtherScale;
  private int curGoalMode;
  private double curGoalOtherScale;
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
      boolean inverted,
      double gearRatio,
      double minDeg,
      double maxDeg,
      Defaults defaults,
      NetworkTable nt) {
    this.name = name;
    this.baseMinDeg = minDeg;
    this.baseMaxDeg = maxDeg;
    this.ioSign = inverted ? -1.0 : 1.0;
    this.accelLimitRadPerS2 = Math.max(0.0, defaults.maxAcc);
    this.defaultsCfg = defaults;
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
    setPosDeg = nt.getEntry("SetPosDeg");
    setPos = nt.getEntry("SetPos");
    resetDefaults = nt.getEntry("ResetDefaults");
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
    ntGoalRelativeToOther = nt.getEntry("GoalRelativeToOther");
    ntkG2 = nt.getEntry("kG2");
    ntkG2OtherScale = nt.getEntry("kG2OtherScale");
    ntGlobalOtherScale = nt.getEntry("GlobalOtherScale");
    ntGoalMode = nt.getEntry("GoalMode");
    ntGoalOtherScale = nt.getEntry("GoalOtherScale");
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
    ntOtherAngleDeg = nt.getEntry("OtherAngleDeg");
    ntAngleDegGlobal = nt.getEntry("AngleDegGlobal");
    ntGoalDegGlobal = nt.getEntry("GoalDegGlobal");

    // Defaults (publish once)
    targetDeg.setDouble(0.0);
    zero.setBoolean(false);
    setPosDeg.setDouble(0.0);
    setPos.setBoolean(false);
    resetDefaults.setBoolean(false);
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
    curGoalRelativeToOther = defaults.useGlobalAngleForGravity;
    curkG2 = defaults.kg2;
    curkG2OtherScale = defaults.kg2OtherScale;
    curGlobalOtherScale = defaults.globalOtherScale;
    curGoalMode = defaults.goalMode;
    curGoalOtherScale = defaults.goalOtherScale;
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
    ntGoalRelativeToOther.setBoolean(curGoalRelativeToOther);
    ntkG2.setDouble(curkG2);
    ntkG2OtherScale.setDouble(curkG2OtherScale);
    ntGlobalOtherScale.setDouble(curGlobalOtherScale);
    ntGoalMode.setDouble(curGoalMode);
    ntGoalOtherScale.setDouble(curGoalOtherScale);
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
    ntOtherAngleDeg.setDouble(0.0);
    ntAngleDegGlobal.setDouble(0.0);
    ntGoalDegGlobal.setDouble(0.0);
  }

  private void applyDefaultsToNtAndController() {
    // Apply to internal state
    curkP = defaultsCfg.kp;
    curkI = defaultsCfg.ki;
    curkD = defaultsCfg.kd;
    curKs = defaultsCfg.ks;
    curKv = defaultsCfg.kv;
    curKa = defaultsCfg.ka;
    curKg = defaultsCfg.kg;
    curGoalRelativeToOther = defaultsCfg.useGlobalAngleForGravity;
    curkG2 = defaultsCfg.kg2;
    curkG2OtherScale = defaultsCfg.kg2OtherScale;
    curGlobalOtherScale = defaultsCfg.globalOtherScale;
    curGoalMode = defaultsCfg.goalMode;
    curGoalOtherScale = defaultsCfg.goalOtherScale;
    curMaxVolts = defaultsCfg.maxVolts;
    curIZoneDeg = defaultsCfg.iZoneDeg;
    curIMaxVolts = defaultsCfg.iMaxVolts;

    // Publish to NT
    ntkP.setDouble(curkP);
    ntkI.setDouble(curkI);
    ntkD.setDouble(curkD);
    ntKs.setDouble(curKs);
    ntKv.setDouble(curKv);
    ntKa.setDouble(curKa);
    ntKg.setDouble(curKg);
    ntGoalRelativeToOther.setBoolean(curGoalRelativeToOther);
    ntkG2.setDouble(curkG2);
    ntkG2OtherScale.setDouble(curkG2OtherScale);
    ntGlobalOtherScale.setDouble(curGlobalOtherScale);
    ntGoalMode.setDouble(curGoalMode);
    ntGoalOtherScale.setDouble(curGoalOtherScale);
    ntMaxVolts.setDouble(curMaxVolts);
    ntIZoneDeg.setDouble(curIZoneDeg);
    ntIMaxVolts.setDouble(curIMaxVolts);

    // Apply to controller
    pid.setPID(curkP, curkI, curkD);
    pid.setIntegratorRange(-curIMaxVolts, curIMaxVolts);
    if (curIZoneDeg <= 0.0) {
      pid.setIZone(Double.POSITIVE_INFINITY);
    } else {
      pid.setIZone(Math.toRadians(curIZoneDeg));
    }
  }

  public String getName() { return name; }
  public double getAngleRad() { return enc.getPosition() * ioSign; }
  public double getAngleDeg() { return Math.toDegrees(getAngleRad()); }

  public void zeroNow() {
    enc.setPosition(0.0);
    pid.reset(0.0);
    prevSpVel = 0.0;
    lastDir = 0.0;
  }

  /** When not the selected joint: hold current angle and output 0V. */
  public void inactive() {
    // Still process one-shot commands even when inactive (so web reset/zero/setpos always works).
    if (resetDefaults.getBoolean(false)) {
      applyDefaultsToNtAndController();
      resetDefaults.setBoolean(false);
    }
    if (zero.getBoolean(false)) {
      zeroNow();
      targetDeg.setDouble(0.0);
      testEnable.setBoolean(false);
      zero.setBoolean(false);
    }
    if (setPos.getBoolean(false)) {
      double deg = setPosDeg.getDouble(0.0);
      double rad = Math.toRadians(deg);
      enc.setPosition(rad * ioSign);
      pid.reset(rad);
      prevSpVel = 0.0;
      lastDir = 0.0;
      targetDeg.setDouble(deg);
      testEnable.setBoolean(false);
      setPos.setBoolean(false);
    }
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
    update(nowSec, Double.NaN);
  }

  /** Update with a coupled "other joint" angle (rad). Pass NaN if unused. */
  public void update(double nowSec, double otherAngleRad) {
    boolean enabled = DriverStation.isEnabled();
    ntActive.setBoolean(true);

    // One-shot reset defaults (from Constants)
    if (resetDefaults.getBoolean(false)) {
      applyDefaultsToNtAndController();
      resetDefaults.setBoolean(false);
    }

    // One-shot zero
    if (zero.getBoolean(false)) {
      zeroNow();
      // Prevent immediate motion: make current position the manual target.
      targetDeg.setDouble(0.0);
      // Safety: disable sine test when calibrating.
      testEnable.setBoolean(false);
      zero.setBoolean(false);
    }

    // One-shot set position (deg)
    if (setPos.getBoolean(false)) {
      double deg = setPosDeg.getDouble(0.0);
      double rad = Math.toRadians(deg);
      // set raw encoder so that getAngleRad() becomes rad
      enc.setPosition(rad * ioSign);
      pid.reset(rad);
      prevSpVel = 0.0;
      lastDir = 0.0;
      // Prevent immediate motion: make current position the manual target.
      targetDeg.setDouble(deg);
      // Safety: disable sine test when calibrating.
      testEnable.setBoolean(false);
      setPos.setBoolean(false);
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
      curGoalRelativeToOther = ntGoalRelativeToOther.getBoolean(curGoalRelativeToOther);
      curkG2 = ntkG2.getDouble(curkG2);
      curkG2OtherScale = ntkG2OtherScale.getDouble(curkG2OtherScale);
      curGlobalOtherScale = ntGlobalOtherScale.getDouble(curGlobalOtherScale);
      curGoalMode = (int) ntGoalMode.getDouble(curGoalMode);
      curGoalOtherScale = ntGoalOtherScale.getDouble(curGoalOtherScale);
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
    double otherDeg = Double.isFinite(otherAngleRad) ? Math.toDegrees(otherAngleRad) : 0.0;
    double localGoalDeg = goalDeg;
    if (Double.isFinite(otherAngleRad)) {
      if (curGoalMode == 1) {
        // Follow/add other: local goal = goal + scale*other (set goal=0, scale=1 => local=other)
        localGoalDeg = goalDeg + curGoalOtherScale * otherDeg;
      } else if (curGoalMode == 2) {
        // Global mode: treat goalDeg as global desired => local = goal - scale*other
        localGoalDeg = goalDeg - curGoalOtherScale * otherDeg;
      }
    }
    localGoalDeg = MathUtil.clamp(localGoalDeg, curMinDeg, curMaxDeg);
    pid.setGoal(Math.toRadians(localGoalDeg));

    // Telemetry (inputs)
    ntEnabled.setBoolean(enabled);
    double selfDeg = getAngleDeg();
    ntAngleDeg.setDouble(selfDeg);
    ntGoalDeg.setDouble(localGoalDeg); // local joint goal actually used
    ntInputTargetDeg.setDouble(targetDeg.getDouble(0.0));
    ntOtherAngleDeg.setDouble(otherDeg);
    // Always publish global angle/goal for debugging; if other is NaN, it's just local.
    double globalDeg = Double.isFinite(otherAngleRad) ? selfDeg + curGlobalOtherScale * otherDeg : selfDeg;
    double globalGoalDeg = Double.isFinite(otherAngleRad) ? localGoalDeg + curGlobalOtherScale * otherDeg : localGoalDeg;
    ntAngleDegGlobal.setDouble(globalDeg);
    ntGoalDegGlobal.setDouble(globalGoalDeg);

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
    // IMPORTANT: Gravity depends on *actual* angle, not setpoint. Using setpoint can cause brief pulses when
    // the setpoint jumps (e.g., target=90deg => cos=0) even if the arm hasn't moved yet.
    double selfAngleRad = getAngleRad();
    double gravAngleRad =
        (curGoalRelativeToOther && Double.isFinite(otherAngleRad))
            ? selfAngleRad + curGlobalOtherScale * otherAngleRad
            : selfAngleRad;
    double gVolts = curKg * Math.cos(gravAngleRad);
    if (Double.isFinite(otherAngleRad) && Math.abs(curkG2) > 1e-9) {
      gVolts += curkG2 * Math.cos(selfAngleRad + curkG2OtherScale * otherAngleRad);
    }

    double ffVolts =
        curKs * sign
            + gVolts
            + curKv * vel
            // Ka 不做反向：只沿 sign 方向提供加速度前馈
            + curKa * Math.abs(accel) * sign;
    double volts = pid.calculate(getAngleRad()) + ffVolts;
    lastOutVolts = MathUtil.clamp(volts, -curMaxVolts, curMaxVolts);
    double applied = lastOutVolts * ioSign;
    motor.setVoltage(applied);
    ntOutVoltsCmd.setDouble(applied);
  }
}



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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.PubSubOption;
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
  /** ROS 轨迹追踪（Scheme 1）：对 RosPos 做 PI（D=0），并单独做速度误差反馈 + vel/acc 前馈。 */
  private final PIDController rosPid;
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

  // --- ROS Scheme B inputs (Jetson -> roboRIO) ---
  private final NetworkTableEntry rosEnable;
  private final NetworkTableEntry rosPosRad;
  private final NetworkTableEntry rosVelRadPerS;
  private final NetworkTableEntry rosAccRadPerS2;
  private final NetworkTableEntry rosSeq; // legacy/compat (double)
  private final IntegerSubscriber rosSeqSub; // preferred (NT4 int64)
  private final NetworkTableEntry rosTimeoutS;
  // --- ROS smoothing (optional) ---
  /** Low-pass time constant for RosVel (seconds). 0 disables. */
  private final NetworkTableEntry rosVelTauS;
  /** Low-pass time constant for RosAcc (seconds). 0 disables. */
  private final NetworkTableEntry rosAccTauS;
  /** Limit d(acc)/dt (rad/s^3). 0 disables. */
  private final NetworkTableEntry rosJerkLimitRadPerS3;
  // --- ROS position shaping (roboRIO-side, recommended) ---
  /**
   * If true: treat RosPos as sampled waypoints and generate a smooth reference trajectory (pos/vel/acc) on
   * the roboRIO. This targets the "离散点阶跃" issue without relying on low-pass filtering.
   */
  private final NetworkTableEntry rosPosShaperEnable;
  /** 2nd-order shaper natural frequency (rad/s). Higher = tighter tracking, less smoothing. */
  private final NetworkTableEntry rosPosShaperWnRadPerS;
  /** 2nd-order shaper damping ratio. 1.0 = critical damping. */
  private final NetworkTableEntry rosPosShaperZeta;
  /**
   * If true: interpret RosAcc as a magnitude and apply the control "sign" (from vel/posErr). This is useful
   * when upstream publishes magnitude-only / boxy acceleration.
   */
  private final NetworkTableEntry rosAccUseAbsSign;
  // --- ROS command shaping (roboRIO-side, optional) ---
  /** If true: jerk-limit a generated (vel,acc) command to avoid step/boxy RosAcc. */
  private final NetworkTableEntry rosShapeEnable;
  /** Acceleration limit for shaping (rad/s^2). */
  private final NetworkTableEntry rosShapeAMaxRadPerS2;
  /** Jerk limit for shaping (rad/s^3). */
  private final NetworkTableEntry rosShapeJMaxRadPerS3;
  // --- Ks gating (reduce low-speed dithering) ---
  /** Minimum |velCmd| (rad/s) to apply Ks. */
  private final NetworkTableEntry ksMinVelRadPerS;
  /** Minimum |posErr| (deg) to apply Ks if vel is small. */
  private final NetworkTableEntry ksMinErrDeg;
  /** If true: when a frame is late but still within RosValid timeout, extrapolate using last pos/vel/acc. */
  private final NetworkTableEntry rosPredictEnable;
  /** Max prediction horizon (seconds) to avoid runaway extrapolation. */
  private final NetworkTableEntry rosPredictMaxS;

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
  // --- ROS Scheme B telemetry (roboRIO -> Jetson/Web) ---
  private final NetworkTableEntry ntRosValid, ntRosAgeS, ntRosSeqEcho;
  /** "manual" | "sine" | "ros" | "inactive" | "monitor" */
  private final NetworkTableEntry ntControlMode;
  /** Human-readable reason why output is forced to 0V. */
  private final NetworkTableEntry ntInhibitReason;
  // ROS smoothing telemetry
  private final NetworkTableEntry ntRosVelFiltRadPerS, ntRosAccFiltRadPerS2;
  // ROS prediction telemetry
  private final NetworkTableEntry ntRosPredictDtS;
  // ROS tracking debug breakdown (helps explain OutVoltsCmd shapes)
  private final NetworkTableEntry ntActualVelRadPerS;
  private final NetworkTableEntry ntActualVelFiltRadPerS;
  private final NetworkTableEntry ntVelErrRadPerS;
  private final NetworkTableEntry ntPidPosVolts;
  private final NetworkTableEntry ntVelFbVolts;
  private final NetworkTableEntry ntFfVolts;
  /** Low-pass time constant for measured velocity used in vel error feedback (seconds). 0 disables. */
  private final NetworkTableEntry rosActualVelTauS;
  private double actualVelFiltRadPerS = 0.0;
  // Debug/diagnostics for "early stop" misconceptions
  private final NetworkTableEntry ntSoftLimitClamped; // boolean
  private final NetworkTableEntry ntSaturated; // boolean
  private final NetworkTableEntry ntOutVoltsRaw; // double (before MaxVolts clamp)
  private final NetworkTableEntry ntNowSec; // heartbeat timestamp (seconds)

  // NT4 high-rate publishers (50Hz). Entry publishing often defaults to ~0.1s.
  private final DoublePublisher pubNowSec, pubAngleDeg, pubGoalDeg, pubOutVoltsCmd;
  // Dedicated 100Hz telemetry topics (avoid being “dragged” by subscriber periodic / other publishers)
  private final DoublePublisher pubNowSec100, pubAngleDeg100;
  private final IntegerPublisher pubAngleSeq100;
  private final BooleanPublisher pubEnabled;
  private long angleSeq100 = 0;

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

  // ROS watchdog state
  private long lastRosSeq = Long.MIN_VALUE;
  private double lastRosUpdateSec = -1.0;
  private double lastRosPosRad = 0.0;
  private double lastRosVelRadPerS = 0.0;
  private double lastRosAccRadPerS2 = 0.0;
  // Optional smoothing state (only used when RosVelTauS/RosAccTauS > 0)
  private double rosVelFiltRadPerS = 0.0;
  private double rosAccFiltRadPerS2 = 0.0;
  // Shaping state (roboRIO-side continuous vel/acc)
  private double rosVelCmdRadPerS = 0.0;
  private double rosAccCmdRadPerS2 = 0.0;
  // Position shaper state (roboRIO-side continuous pos/vel/acc)
  private boolean rosPosCmdInit = false;
  private double rosPosCmdRad = 0.0;
  private double rosPosCmdVelRadPerS = 0.0;
  private double rosPosCmdAccRadPerS2 = 0.0;

  // sign selection hysteresis / thresholds (degrees)
  private static final double SIGN_VEL_EPS_RAD_PER_S = 1e-4;
  private static final double SIGN_ERR_DEADBAND_DEG = 0.25;
  private static final double SIGN_ERR_FLIP_DEG = 0.75; // suggested 0.5~1.0deg

  private static void setDefaultBoolean(NetworkTableEntry e, boolean v) {
    try {
      var m = e.getClass().getMethod("setDefaultBoolean", boolean.class);
      m.invoke(e, v);
    } catch (Exception ignored) {
      // Do nothing (avoid overwriting external writers).
    }
  }

  private static void setDefaultDouble(NetworkTableEntry e, double v) {
    try {
      var m = e.getClass().getMethod("setDefaultDouble", double.class);
      m.invoke(e, v);
    } catch (Exception ignored) {
      // Do nothing (avoid overwriting external writers).
    }
  }

  private static void setDefaultString(NetworkTableEntry e, String v) {
    try {
      var m = e.getClass().getMethod("setDefaultString", String.class);
      m.invoke(e, v);
    } catch (Exception ignored) {
      // Do nothing (avoid overwriting external writers).
    }
  }

  /** Read-only: whether an external client is requesting ROS control for this joint. */
  public boolean isRosEnableRequested() {
    return rosEnable.getBoolean(false);
  }

  /** 100Hz telemetry hook (called from ArmSubsystem Notifier). */
  public void publishFastTelemetry(double nowSec) {
    // Legacy topics (still updated here)
    pubNowSec.set(nowSec);
    pubAngleDeg.set(getAngleDeg());
    // Dedicated 100Hz topics (proof-of-rate)
    pubNowSec100.set(nowSec);
    pubAngleDeg100.set(getAngleDeg());
    pubAngleSeq100.set(++angleSeq100);
  }

  private double computeSign(double velRadPerS, double posErrRad) {
    if (Math.abs(velRadPerS) > SIGN_VEL_EPS_RAD_PER_S) {
      double s = Math.signum(velRadPerS);
      if (s != 0.0) lastDir = s;
      return s;
    }
    double errDeg = Math.abs(Math.toDegrees(posErrRad));
    double errSign = Math.signum(posErrRad);
    double sign;
    if (lastDir == 0.0) {
      sign = (errDeg >= SIGN_ERR_FLIP_DEG) ? errSign : 0.0;
    } else if (errDeg < SIGN_ERR_DEADBAND_DEG) {
      sign = lastDir;
    } else if (errSign == lastDir) {
      sign = lastDir;
    } else {
      sign = (errDeg >= SIGN_ERR_FLIP_DEG) ? errSign : lastDir;
    }
    if (sign != 0.0) lastDir = sign;
    return sign;
  }

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
    // IMPORTANT: Encoder position on Spark is updated via CAN status frames.
    // If this is slow (e.g. ~20Hz), then AngleDeg will appear "stair-stepped" even if we publish at 100Hz,
    // because enc.getPosition() won't change between frames and NT4 will suppress duplicates.
    // Raise the primary encoder status frame rate so AngleDeg100 can actually update at ~100Hz.
    cfg.signals.primaryEncoderPositionPeriodMs((int) Math.round(ArmConstants.NT_ANGLE_PERIOD_S * 1000.0));
    cfg.signals.primaryEncoderPositionAlwaysOn(true);
    // Velocity is optional but useful for diagnostics/FF; keep it reasonably fast without spamming CAN.
    cfg.signals.primaryEncoderVelocityPeriodMs(20);
    cfg.signals.primaryEncoderVelocityAlwaysOn(true);
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

    // ROS 轨迹追踪：PI on position. (Velocity error feedback handled separately via curkD)
    rosPid = new PIDController(defaults.kp, defaults.ki, 0.0, ArmConstants.DT);
    rosPid.setTolerance(Math.toRadians(1.0));
    rosPid.reset();

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

    rosEnable = nt.getEntry("RosEnable");
    rosPosRad = nt.getEntry("RosPosRad");
    rosVelRadPerS = nt.getEntry("RosVelRadPerS");
    rosAccRadPerS2 = nt.getEntry("RosAccRadPerS2");
    rosSeq = nt.getEntry("RosSeq");
    rosSeqSub = nt.getIntegerTopic("RosSeq").subscribe(0, PubSubOption.periodic(ArmConstants.NT_FAST_PERIOD_S));
    rosTimeoutS = nt.getEntry("RosTimeoutS");
    rosVelTauS = nt.getEntry("RosVelTauS");
    rosAccTauS = nt.getEntry("RosAccTauS");
    rosJerkLimitRadPerS3 = nt.getEntry("RosJerkLimitRadPerS3");
    rosPosShaperEnable = nt.getEntry("RosPosShaperEnable");
    rosPosShaperWnRadPerS = nt.getEntry("RosPosShaperWnRadPerS");
    rosPosShaperZeta = nt.getEntry("RosPosShaperZeta");
    rosAccUseAbsSign = nt.getEntry("RosAccUseAbsSign");
    rosPredictEnable = nt.getEntry("RosPredictEnable");
    rosPredictMaxS = nt.getEntry("RosPredictMaxS");
    rosShapeEnable = nt.getEntry("RosShapeEnable");
    rosShapeAMaxRadPerS2 = nt.getEntry("RosShapeAMaxRadPerS2");
    rosShapeJMaxRadPerS3 = nt.getEntry("RosShapeJMaxRadPerS3");
    ksMinVelRadPerS = nt.getEntry("KsMinVelRadPerS");
    ksMinErrDeg = nt.getEntry("KsMinErrDeg");

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
    ntRosValid = nt.getEntry("RosValid");
    ntRosAgeS = nt.getEntry("RosAgeS");
    ntRosSeqEcho = nt.getEntry("RosSeqEcho");
    ntControlMode = nt.getEntry("ControlMode");
    ntInhibitReason = nt.getEntry("InhibitReason");
    ntRosVelFiltRadPerS = nt.getEntry("RosVelFiltRadPerS");
    ntRosAccFiltRadPerS2 = nt.getEntry("RosAccFiltRadPerS2");
    ntRosPredictDtS = nt.getEntry("RosPredictDtS");
    ntActualVelRadPerS = nt.getEntry("ActualVelRadPerS");
    ntActualVelFiltRadPerS = nt.getEntry("ActualVelFiltRadPerS");
    ntVelErrRadPerS = nt.getEntry("VelErrRadPerS");
    ntPidPosVolts = nt.getEntry("PidPosVolts");
    ntVelFbVolts = nt.getEntry("VelFbVolts");
    ntFfVolts = nt.getEntry("FfVolts");
    rosActualVelTauS = nt.getEntry("RosActualVelTauS");
    ntSoftLimitClamped = nt.getEntry("SoftLimitClamped");
    ntSaturated = nt.getEntry("Saturated");
    ntOutVoltsRaw = nt.getEntry("OutVoltsRaw");
    ntNowSec = nt.getEntry("NowSec");

    // Angle/NowSec use a faster publisher period; actual "update rate" depends on how often we call set().
    pubNowSec = nt.getDoubleTopic("NowSec").publish(PubSubOption.periodic(ArmConstants.NT_ANGLE_PERIOD_S));
    pubAngleDeg = nt.getDoubleTopic("AngleDeg").publish(PubSubOption.periodic(ArmConstants.NT_ANGLE_PERIOD_S));
    // Dedicated 100Hz topics (no other code path should write these)
    pubNowSec100 = nt.getDoubleTopic("NowSec100").publish(PubSubOption.periodic(ArmConstants.NT_ANGLE_PERIOD_S));
    pubAngleDeg100 = nt.getDoubleTopic("AngleDeg100").publish(PubSubOption.periodic(ArmConstants.NT_ANGLE_PERIOD_S));
    pubAngleSeq100 = nt.getIntegerTopic("AngleSeq100").publish(PubSubOption.periodic(ArmConstants.NT_ANGLE_PERIOD_S));
    pubGoalDeg = nt.getDoubleTopic("GoalDeg").publish(PubSubOption.periodic(ArmConstants.NT_FAST_PERIOD_S));
    pubOutVoltsCmd =
        nt.getDoubleTopic("OutVoltsCmd").publish(PubSubOption.periodic(ArmConstants.NT_FAST_PERIOD_S));
    pubEnabled = nt.getBooleanTopic("Enabled").publish(PubSubOption.periodic(ArmConstants.NT_FAST_PERIOD_S));

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

    // ROS defaults (do not overwrite external writers)
    setDefaultBoolean(rosEnable, false);
    setDefaultDouble(rosPosRad, 0.0);
    setDefaultDouble(rosVelRadPerS, 0.0);
    setDefaultDouble(rosAccRadPerS2, 0.0);
    setDefaultDouble(rosSeq, 0.0);
    // Recommended starting range: 0.25~0.40s (avoid too small); watchdog floor is enforced in code.
    setDefaultDouble(rosTimeoutS, 0.30);
    // Default: do NOT enable low-pass filtering.
    setDefaultDouble(rosVelTauS, 0.0);
    setDefaultDouble(rosAccTauS, 0.0);
    setDefaultDouble(rosJerkLimitRadPerS3, 0.0);
    // Position shaper defaults: OFF (user prefers raw RosPos tracking without shaping).
    setDefaultBoolean(rosPosShaperEnable, false);
    setDefaultDouble(rosPosShaperWnRadPerS, 20.0);
    setDefaultDouble(rosPosShaperZeta, 1.0);
    // Acc sign mode default: robust with magnitude-only RosAcc
    setDefaultBoolean(rosAccUseAbsSign, true);
    // Prediction defaults: bridge/网络偶发 40~50ms gap 时更丝滑
    setDefaultBoolean(rosPredictEnable, true);
    setDefaultDouble(rosPredictMaxS, 0.06);
    // RoboRIO-side shaping defaults (start disabled; enable if RosAcc looks boxy)
    setDefaultBoolean(rosShapeEnable, false);
    setDefaultDouble(rosShapeAMaxRadPerS2, 3.0);
    setDefaultDouble(rosShapeJMaxRadPerS3, 10.0);
    // Ks gating defaults (helps low-speed stick-slip)
    setDefaultDouble(ksMinVelRadPerS, 0.05);
    setDefaultDouble(ksMinErrDeg, 0.20);

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
    setDefaultString(ntGoalSource, "manual");
    ntActive.setBoolean(true);
    ntOtherAngleDeg.setDouble(0.0);
    ntAngleDegGlobal.setDouble(0.0);
    ntGoalDegGlobal.setDouble(0.0);
    setDefaultDouble(ntNowSec, 0.0);
    setDefaultBoolean(ntRosValid, false);
    setDefaultDouble(ntRosAgeS, 1e9);
    setDefaultDouble(ntRosSeqEcho, 0.0);
    setDefaultString(ntControlMode, "manual");
    setDefaultString(ntInhibitReason, "");
    setDefaultDouble(ntRosVelFiltRadPerS, 0.0);
    setDefaultDouble(ntRosAccFiltRadPerS2, 0.0);
    setDefaultDouble(ntRosPredictDtS, 0.0);
    setDefaultDouble(ntActualVelRadPerS, 0.0);
    setDefaultDouble(ntActualVelFiltRadPerS, 0.0);
    setDefaultDouble(ntVelErrRadPerS, 0.0);
    setDefaultDouble(ntPidPosVolts, 0.0);
    setDefaultDouble(ntVelFbVolts, 0.0);
    setDefaultDouble(ntFfVolts, 0.0);
    // Default: no measured-velocity low-pass filtering (keep it honest; smoothing is done via shaper).
    setDefaultDouble(rosActualVelTauS, 0.0);
    setDefaultBoolean(ntSoftLimitClamped, false);
    setDefaultBoolean(ntSaturated, false);
    setDefaultDouble(ntOutVoltsRaw, 0.0);
    pubNowSec.set(0.0);
    pubAngleDeg.set(0.0);
    pubGoalDeg.set(0.0);
    pubOutVoltsCmd.set(0.0);
    pubEnabled.set(false);
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
    rosPid.setPID(curkP, curkI, 0.0);
    pid.setIntegratorRange(-curIMaxVolts, curIMaxVolts);
    rosPid.setIntegratorRange(-curIMaxVolts, curIMaxVolts);
    if (curIZoneDeg <= 0.0) {
      pid.setIZone(Double.POSITIVE_INFINITY);
      rosPid.setIZone(Double.POSITIVE_INFINITY);
    } else {
      pid.setIZone(Math.toRadians(curIZoneDeg));
      rosPid.setIZone(Math.toRadians(curIZoneDeg));
    }
  }

  public String getName() { return name; }
  public double getAngleRad() { return enc.getPosition() * ioSign; }
  public double getAngleDeg() { return Math.toDegrees(getAngleRad()); }

  public void zeroNow() {
    enc.setPosition(0.0);
    pid.reset(0.0);
    rosPid.reset();
    prevSpVel = 0.0;
    lastDir = 0.0;
    lastRosPosRad = 0.0;
    lastRosVelRadPerS = 0.0;
    lastRosAccRadPerS2 = 0.0;
    rosVelFiltRadPerS = 0.0;
    rosAccFiltRadPerS2 = 0.0;
    actualVelFiltRadPerS = 0.0;
    rosVelCmdRadPerS = 0.0;
    rosAccCmdRadPerS2 = 0.0;
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
      rosPid.reset();
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
    boolean enabled = DriverStation.isEnabled();
    ntEnabled.setBoolean(enabled);
    pubEnabled.set(enabled);
    double selfDeg = getAngleDeg();
    ntAngleDeg.setDouble(selfDeg);
    pubAngleDeg.set(selfDeg);
    ntOutVoltsCmd.setDouble(0.0);
    pubOutVoltsCmd.set(0.0);
    ntControlMode.setString("inactive");
    ntInhibitReason.setString("inactive");
  }

  /** Called by ArmSubsystem every 20ms. */
  public void update(double nowSec) {
    update(nowSec, Double.NaN, false);
  }

  /** Update with a coupled "other joint" angle (rad). Pass NaN if unused. */
  public void update(double nowSec, double otherAngleRad) {
    update(nowSec, otherAngleRad, false);
  }

  /** Update with other joint angle + monitor-only flag. */
  public void update(double nowSec, double otherAngleRad, boolean monitorOnly) {
    boolean enabled = DriverStation.isEnabled();
    ntActive.setBoolean(true);
    ntEnabled.setBoolean(enabled);
    pubEnabled.set(enabled);
    ntNowSec.setDouble(nowSec);
    pubNowSec.set(nowSec);

    // One-shot reset defaults (from Constants)
    if (resetDefaults.getBoolean(false)) {
      applyDefaultsToNtAndController();
      resetDefaults.setBoolean(false);
    }

    // One-shot zero
    if (zero.getBoolean(false)) {
      zeroNow();
      if (!monitorOnly) {
        // Prevent immediate motion: make current position the manual target.
        targetDeg.setDouble(0.0);
        // Safety: disable sine test when calibrating.
        testEnable.setBoolean(false);
      }
      zero.setBoolean(false);
    }

    // One-shot set position (deg)
    if (setPos.getBoolean(false)) {
      double deg = setPosDeg.getDouble(0.0);
      double rad = Math.toRadians(deg);
      // set raw encoder so that getAngleRad() becomes rad
      enc.setPosition(rad * ioSign);
      pid.reset(rad);
      rosPid.reset();
      prevSpVel = 0.0;
      lastDir = 0.0;
      if (!monitorOnly) {
        // Prevent immediate motion: make current position the manual target.
        targetDeg.setDouble(deg);
        // Safety: disable sine test when calibrating.
        testEnable.setBoolean(false);
      }
      setPos.setBoolean(false);
      lastRosPosRad = rosPosRad.getDouble(0.0);
      lastRosVelRadPerS = rosVelRadPerS.getDouble(0.0);
      lastRosAccRadPerS2 = rosAccRadPerS2.getDouble(0.0);
      rosVelFiltRadPerS = 0.0;
      rosAccFiltRadPerS2 = 0.0;
      actualVelFiltRadPerS = 0.0;
      rosVelCmdRadPerS = 0.0;
      rosAccCmdRadPerS2 = 0.0;
      rosPosCmdInit = false;
      rosPosCmdRad = 0.0;
      rosPosCmdVelRadPerS = 0.0;
      rosPosCmdAccRadPerS2 = 0.0;
    }

    // Monitor-only: publish telemetry but never drive motors.
    if (monitorOnly) {
      ntEnabled.setBoolean(enabled);
      pubEnabled.set(enabled);
      double selfDeg = getAngleDeg();
      ntAngleDeg.setDouble(selfDeg);
      pubAngleDeg.set(selfDeg);
      ntInputTargetDeg.setDouble(targetDeg.getDouble(0.0));
      double otherDeg = Double.isFinite(otherAngleRad) ? Math.toDegrees(otherAngleRad) : 0.0;
      ntOtherAngleDeg.setDouble(otherDeg);
      double globalDeg = Double.isFinite(otherAngleRad) ? selfDeg + curGlobalOtherScale * otherDeg : selfDeg;
      ntAngleDegGlobal.setDouble(globalDeg);
      ntGoalSource.setString(testEnable.getBoolean(false) ? "sine" : "manual");
      pubGoalDeg.set(ntGoalDeg.getDouble(0.0));
      ntGoalDegGlobal.setDouble(ntGoalDegGlobal.getDouble(0.0));
      motor.setVoltage(0.0);
      lastOutVolts = 0.0;
      ntOutVoltsCmd.setDouble(0.0);
      pubOutVoltsCmd.set(0.0);
      ntControlMode.setString("monitor");
      ntInhibitReason.setString("monitorOnly");
      return;
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

    // --- ROS Scheme B: external setpoint tracking (pos/vel/acc) ---
    boolean rosOn = rosEnable.getBoolean(false);
    if (rosOn) {
      // Watchdog based on RosSeq updates
      // Minimum timeout floor to avoid "stop and go" due to jitter.
      double timeout = Math.max(rosTimeoutS.getDouble(0.30), 0.20);
      long seq = rosSeqSub.get();
      if (seq == 0) seq = (long) rosSeq.getDouble(0.0); // legacy fallback
      if (seq != lastRosSeq) {
        lastRosSeq = seq;
        lastRosUpdateSec = nowSec;
        // Sample a coherent frame (pos/vel/acc) only when seq changes.
        lastRosPosRad = rosPosRad.getDouble(0.0);
        lastRosVelRadPerS = rosVelRadPerS.getDouble(0.0);
        lastRosAccRadPerS2 = rosAccRadPerS2.getDouble(0.0);
      }
      double age = (lastRosUpdateSec < 0.0) ? 1e9 : (nowSec - lastRosUpdateSec);
      boolean rosValid = enabled && age <= timeout;
      ntRosValid.setBoolean(rosValid);
      ntRosAgeS.setDouble(age);
      ntRosSeqEcho.setDouble((double) lastRosSeq);
      ntControlMode.setString("rosTrack");

      // Scheme 1: Trajectory tracking.
      // Treat RosPos/RosVel/RosAcc as reference trajectory (NOT a final goal to replan).
      double desiredPosRaw = lastRosPosRad;
      double desiredVel = lastRosVelRadPerS;
      double desiredAcc = lastRosAccRadPerS2;
      // Optional prediction across small gaps for smoothness (uses last pos/vel/acc)
      double predDt = 0.0;
      if (rosValid && rosPredictEnable.getBoolean(true)) {
        double maxPred = Math.max(0.0, rosPredictMaxS.getDouble(0.06));
        predDt = MathUtil.clamp(age, 0.0, maxPred);
        desiredPosRaw = desiredPosRaw + desiredVel * predDt + 0.5 * desiredAcc * predDt * predDt;
        desiredVel = desiredVel + desiredAcc * predDt;
      }
      ntRosPredictDtS.setDouble(predDt);

      // Clamp desired position to soft limits (local joint coords)
      double minRad = Math.toRadians(curMinDeg);
      double maxRad = Math.toRadians(curMaxDeg);
      double posTargetRad = MathUtil.clamp(desiredPosRaw, minRad, maxRad);
      boolean clamped = posTargetRad != desiredPosRaw;
      ntSoftLimitClamped.setBoolean(clamped);

      // === “平滑”位置整形器：把离散 RosPos 变成连续 pos/vel/acc 参考轨迹 ===
      // NOTE: 不是低通滤波，而是生成一个动态系统的连续参考（更像“函数”轨迹）。
      boolean posShape = rosPosShaperEnable.getBoolean(true);
      double posRefRad = posTargetRad;
      if (rosValid && posShape) {
        double wn = Math.max(1e-3, rosPosShaperWnRadPerS.getDouble(20.0));
        double zeta = Math.max(0.1, rosPosShaperZeta.getDouble(1.0));
        if (!rosPosCmdInit) {
          rosPosCmdInit = true;
          rosPosCmdRad = posRefRad;
          rosPosCmdVelRadPerS = desiredVel;
          rosPosCmdAccRadPerS2 = 0.0;
        }
        double x1 = rosPosCmdRad;
        double x2 = rosPosCmdVelRadPerS;
        double x2dot = (-2.0 * zeta * wn * x2) - (wn * wn * (x1 - posRefRad));
        double x1dot = x2;
        x1 += ArmConstants.DT * x1dot;
        x2 += ArmConstants.DT * x2dot;
        x1 = MathUtil.clamp(x1, minRad, maxRad);
        rosPosCmdRad = x1;
        rosPosCmdVelRadPerS = x2;
        rosPosCmdAccRadPerS2 = x2dot;
        posRefRad = rosPosCmdRad;
        desiredVel = rosPosCmdVelRadPerS;
        desiredAcc = rosPosCmdAccRadPerS2;
      } else {
        rosPosCmdInit = false;
      }

      // (Optional) Legacy vel/acc filtering (default OFF). Skip when position shaper is enabled.
      if (!posShape) {
        double velTau = Math.max(0.0, rosVelTauS.getDouble(0.0));
        double accTau = Math.max(0.0, rosAccTauS.getDouble(0.0));
        if (velTau > 1e-6) {
          double a = ArmConstants.DT / (velTau + ArmConstants.DT);
          rosVelFiltRadPerS = rosVelFiltRadPerS + a * (desiredVel - rosVelFiltRadPerS);
          desiredVel = rosVelFiltRadPerS;
        } else {
          rosVelFiltRadPerS = desiredVel;
        }
        if (accTau > 1e-6) {
          double a = ArmConstants.DT / (accTau + ArmConstants.DT);
          rosAccFiltRadPerS2 = rosAccFiltRadPerS2 + a * (desiredAcc - rosAccFiltRadPerS2);
          desiredAcc = rosAccFiltRadPerS2;
        } else {
          rosAccFiltRadPerS2 = desiredAcc;
        }
        double jerk = Math.max(0.0, rosJerkLimitRadPerS3.getDouble(0.0));
        if (jerk > 1e-6) {
          double maxDelta = jerk * ArmConstants.DT;
          double delta = desiredAcc - rosAccFiltRadPerS2;
          if (delta > maxDelta) desiredAcc = rosAccFiltRadPerS2 + maxDelta;
          else if (delta < -maxDelta) desiredAcc = rosAccFiltRadPerS2 - maxDelta;
          rosAccFiltRadPerS2 = desiredAcc;
        }
      }

      // Optional roboRIO-side vel/acc shaping (default OFF). Skip when position shaper is enabled.
      if (!posShape && rosShapeEnable.getBoolean(false)) {
        double aMax = Math.max(0.0, rosShapeAMaxRadPerS2.getDouble(0.0));
        double jMax = Math.max(0.0, rosShapeJMaxRadPerS3.getDouble(0.0));
        double accDes = (desiredVel - rosVelCmdRadPerS) / ArmConstants.DT;
        if (aMax > 1e-6) accDes = MathUtil.clamp(accDes, -aMax, aMax);
        if (jMax > 1e-6) {
          double maxDa = jMax * ArmConstants.DT;
          double da = MathUtil.clamp(accDes - rosAccCmdRadPerS2, -maxDa, maxDa);
          rosAccCmdRadPerS2 += da;
        } else {
          rosAccCmdRadPerS2 = accDes;
        }
        rosVelCmdRadPerS += rosAccCmdRadPerS2 * ArmConstants.DT;
        desiredVel = rosVelCmdRadPerS;
        desiredAcc = rosAccCmdRadPerS2;
      } else if (!posShape) {
        rosVelCmdRadPerS = desiredVel;
        rosAccCmdRadPerS2 = desiredAcc;
      }
      // If clamped and command is trying to push further into the limit, zero vel/acc FF.
      if (clamped) {
        if ((posRefRad <= minRad + 1e-9 && desiredVel < 0.0)
            || (posRefRad >= maxRad - 1e-9 && desiredVel > 0.0)) {
          desiredVel = 0.0;
          desiredAcc = 0.0;
          if (posShape) {
            rosPosCmdVelRadPerS = 0.0;
            rosPosCmdAccRadPerS2 = 0.0;
          }
        }
      }

      double goalDeg = Math.toDegrees(posRefRad);
      ntGoalDeg.setDouble(goalDeg);
      pubGoalDeg.set(goalDeg);
      ntGoalSource.setString("ros");

      // Publish angle telemetry
      double selfDeg = getAngleDeg();
      ntAngleDeg.setDouble(selfDeg);
      pubAngleDeg.set(selfDeg);
      double otherDeg = Double.isFinite(otherAngleRad) ? Math.toDegrees(otherAngleRad) : 0.0;
      ntOtherAngleDeg.setDouble(otherDeg);
      double globalDeg = Double.isFinite(otherAngleRad) ? selfDeg + curGlobalOtherScale * otherDeg : selfDeg;
      ntAngleDegGlobal.setDouble(globalDeg);
      double globalGoalDeg = Double.isFinite(otherAngleRad) ? goalDeg + curGlobalOtherScale * otherDeg : goalDeg;
      ntGoalDegGlobal.setDouble(globalGoalDeg);

      // Gate conditions
      if (!enabled) {
        motor.setVoltage(0.0);
        lastOutVolts = 0.0;
        ntOutVoltsCmd.setDouble(0.0);
        pubOutVoltsCmd.set(0.0);
        ntInhibitReason.setString("disabled");
        return;
      }
      if (curMaxVolts <= 1e-6) {
        motor.setVoltage(0.0);
        lastOutVolts = 0.0;
        ntOutVoltsCmd.setDouble(0.0);
        pubOutVoltsCmd.set(0.0);
        ntInhibitReason.setString("maxVolts=0");
        return;
      }
      if (!rosValid) {
        motor.setVoltage(0.0);
        lastOutVolts = 0.0;
        ntOutVoltsCmd.setDouble(0.0);
        pubOutVoltsCmd.set(0.0);
        ntInhibitReason.setString("rosTimeout");
        return;
      }

      // Control
      double selfRad = getAngleRad();
      double actualVelRaw = enc.getVelocity() * ioSign; // rad/s
      double actualVel = actualVelRaw;
      // Optional filtering on measured velocity (helps low-speed "颗粒感" due to velocity quantization/noise)
      double vTau = Math.max(0.0, rosActualVelTauS.getDouble(0.0));
      if (vTau > 1e-6) {
        double a = ArmConstants.DT / (vTau + ArmConstants.DT);
        actualVelFiltRadPerS = actualVelFiltRadPerS + a * (actualVelRaw - actualVelFiltRadPerS);
        actualVel = actualVelFiltRadPerS;
      } else {
        actualVelFiltRadPerS = actualVelRaw;
      }
      double posErr = posRefRad - selfRad;
      double velErr = desiredVel - actualVel;
      // Publish the reference (post prediction/soft-limit) as telemetry.
      ntRosVelFiltRadPerS.setDouble(desiredVel);
      ntRosAccFiltRadPerS2.setDouble(desiredAcc);
      ntActualVelRadPerS.setDouble(actualVelRaw);
      ntActualVelFiltRadPerS.setDouble(actualVel);
      ntVelErrRadPerS.setDouble(velErr);
      double sign = computeSign(desiredVel, posErr);

      // Gravity feedforward uses actual angle (optionally composed with other)
      double gravAngleRad =
          (curGoalRelativeToOther && Double.isFinite(otherAngleRad))
              ? selfRad + curGlobalOtherScale * otherAngleRad
              : selfRad;
      double gVolts = curKg * Math.cos(gravAngleRad);
      if (Double.isFinite(otherAngleRad) && Math.abs(curkG2) > 1e-9) {
        gVolts += curkG2 * Math.cos(selfRad + curkG2OtherScale * otherAngleRad);
      }

      // --- Acc FF sign mode ---
      // If using the position shaper, desiredAcc is already a physically signed value.
      // Otherwise, upstream may publish magnitude-only acc; allow abs(acc)*sign mode to avoid the “末尾一拍”。
      boolean useAbsSignAcc = (!posShape) && rosAccUseAbsSign.getBoolean(true) && !rosShapeEnable.getBoolean(false);
      double accForFf = useAbsSignAcc ? (Math.abs(desiredAcc) * sign) : desiredAcc;

      double ffVolts =
          // Ks gating: avoid low-speed dithering when close to target
          ((Math.abs(desiredVel) >= ksMinVelRadPerS.getDouble(0.05)
                  || Math.abs(Math.toDegrees(posErr)) >= ksMinErrDeg.getDouble(0.20))
              ? (curKs * sign)
              : 0.0)
              + gVolts
              + curKv * desiredVel
              + curKa * accForFf;

      // PI on position + velocity error feedback (use curkD as kD_vel in ROS mode)
      double pidPosVolts = rosPid.calculate(selfRad, posRefRad);
      double velFbVolts = curkD * velErr;
      ntPidPosVolts.setDouble(pidPosVolts);
      ntVelFbVolts.setDouble(velFbVolts);
      ntFfVolts.setDouble(ffVolts);
      double raw = pidPosVolts + velFbVolts + ffVolts;
      ntOutVoltsRaw.setDouble(raw);
      boolean saturated = Math.abs(raw) > curMaxVolts + 1e-9;
      ntSaturated.setBoolean(saturated);

      lastOutVolts = MathUtil.clamp(raw, -curMaxVolts, curMaxVolts);
      double applied = lastOutVolts * ioSign;
      motor.setVoltage(applied);
      ntOutVoltsCmd.setDouble(applied);
      pubOutVoltsCmd.set(applied);
      ntInhibitReason.setString("");
      return;
    } else {
      ntRosValid.setBoolean(false);
      ntRosAgeS.setDouble((lastRosUpdateSec < 0.0) ? 1e9 : (nowSec - lastRosUpdateSec));
      ntRosSeqEcho.setDouble((lastRosSeq == Long.MIN_VALUE) ? 0.0 : (double) lastRosSeq);
      ntSoftLimitClamped.setBoolean(false);
      ntSaturated.setBoolean(false);
    }

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
    pubEnabled.set(enabled);
    double selfDeg = getAngleDeg();
    ntAngleDeg.setDouble(selfDeg);
    pubAngleDeg.set(selfDeg);
    ntGoalDeg.setDouble(localGoalDeg); // local joint goal actually used
    pubGoalDeg.set(localGoalDeg);
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
      pubOutVoltsCmd.set(0.0);
      ntControlMode.setString(test ? "sine" : "manual");
      ntInhibitReason.setString("disabled");
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
    double sign = computeSign(vel, posErr);

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
    pubOutVoltsCmd.set(applied);
    ntControlMode.setString(test ? "sine" : "manual");
    ntInhibitReason.setString(curMaxVolts <= 1e-6 ? "maxVolts=0" : "");
  }
}



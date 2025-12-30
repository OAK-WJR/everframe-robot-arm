// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmJointSubsystem;
import frc.robot.util.SineGoalGenerator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ArmJointSubsystem armJoint = new ArmJointSubsystem();

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("ArmJoint");
  private final NetworkTableEntry targetDeg = table.getEntry("TargetDeg");
  private final NetworkTableEntry zero = table.getEntry("Zero");
  // Sine test (trajectory follow) inputs
  private final NetworkTableEntry testEnable = table.getEntry("TestEnable");
  private final NetworkTableEntry testAmpDeg = table.getEntry("TestAmpDeg");
  private final NetworkTableEntry testPeriodS = table.getEntry("TestPeriodS");
  private final NetworkTableEntry testOffsetDeg = table.getEntry("TestOffsetDeg");
  private boolean prevTestEnable = false;
  private double testStartTime = 0.0;
  private double prevTestAmpDeg = 30.0;
  private double prevTestPeriodS = 4.0;
  private double prevTestOffsetDeg = 0.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Ensure keys exist with defaults so external clients can modify them.
    targetDeg.setDouble(0.0);
    zero.setBoolean(false);
    testEnable.setBoolean(false);
    testAmpDeg.setDouble(30.0);
    testPeriodS.setDouble(4.0);
    testOffsetDeg.setDouble(0.0);

    armJoint.setDefaultCommand(new RunCommand(
        () -> {
          boolean en = testEnable.getBoolean(false);
          double now = Timer.getFPGATimestamp();
          double amp = testAmpDeg.getDouble(30.0);
          double period = testPeriodS.getDouble(4.0);
          double offset = testOffsetDeg.getDouble(0.0);

          if (en && !prevTestEnable) {
            // Start sine test from current time (phase = 0 at start)
            testStartTime = now;
            prevTestAmpDeg = amp;
            prevTestPeriodS = period;
            prevTestOffsetDeg = offset;
          } else if (en) {
            // Keep phase continuous when user changes sine parameters while running
            boolean changed = amp != prevTestAmpDeg || period != prevTestPeriodS || offset != prevTestOffsetDeg;
            if (changed && prevTestPeriodS > 0.0 && period > 0.0) {
              double oldPhase = 2.0 * Math.PI * (now - testStartTime) / prevTestPeriodS;
              testStartTime = now - (oldPhase * period / (2.0 * Math.PI));
              prevTestAmpDeg = amp;
              prevTestPeriodS = period;
              prevTestOffsetDeg = offset;
            }
          }
          prevTestEnable = en;

          if (en) {
            double t = now - testStartTime;
            double goal = SineGoalGenerator.goalDeg(t, offset, amp, period);
            armJoint.setGoalDeg(goal);
          } else {
            armJoint.setGoalDeg(targetDeg.getDouble(0.0));
          }
        },
        armJoint));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(() -> zero.getBoolean(false))
        .onTrue(
            new InstantCommand(() -> { armJoint.zero(); zero.setBoolean(false); }, armJoint)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}

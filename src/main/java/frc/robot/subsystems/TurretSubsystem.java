package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import frc.robot.LimelightHelpers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {

  private final double MAX_ONE_DIR_FOV = 90; // degrees
  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

  // 1 Neo, 6.875 in diameter, 4:1 gearbox, 10:1 pivot gearing, non-continuous
  // 360 deg
  // Total reduction: 4 * 10 = 40:1

  // private ThriftyNova nova = new
  // ThriftyNova(Constants.TurretConstants.kMotorId);

  private static final String LIMELIGHT_NAME = "limelight";
  private static final double TAG_TX_TOLERANCE_DEG = 1.0;
  private static final double MAX_TRACK_STEP_DEG = 3.0;

  private TalonFX falcon = new TalonFX(Constants.TurretConstants.kMotorId);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440))
      .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 10)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withStatorCurrentLimit(Amps.of(10))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new TalonFXWrapper(falcon, DCMotor.getFalcon500(1), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(
          new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretTranslation));

  private Pivot turret = new Pivot(turretConfig);

  public TurretSubsystem() {}

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  public Angle getRobotAdjustedAngle() {
    // Returns the turret angle in the robot's coordinate frame
    // since the turret is mounted backwards, we need to add 180 degrees
    return turret.getAngle().plus(Degrees.of(180));
  }

  public Angle getRawAngle() {
    return turret.getAngle();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command rezero() {
    return Commands.runOnce(() -> falcon.setPosition(0), this).withName("Turret.Rezero");
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  public boolean hasDesiredTag(int... validIds) {
    if (!hasTarget()) return false;

    double seenId = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    for (int id : validIds) {
      if ((int) seenId == id) {
        return true;
      }
    }
    return false;
  }

  public double getTagTx() {
    return LimelightHelpers.getTY(LIMELIGHT_NAME);  //Revert back later
  }

  public Command trackAprilTag(int... validIds) {
    return setAngleDynamic(() -> {
      double currentDeg = turret.getAngle().in(Degrees);
      
      if (!hasDesiredTag(validIds)) {
        return Degrees.of(currentDeg);
      }

      double tx = getTagTx();

      if (Math.abs(tx) < TAG_TX_TOLERANCE_DEG) {
        return Degrees.of(currentDeg);
      }

      double correctionDeg = MathUtil.clamp(tx, -MAX_TRACK_STEP_DEG, MAX_TRACK_STEP_DEG);
      double newSetpointDeg = currentDeg-correctionDeg; 

      newSetpointDeg = MathUtil.clamp(newSetpointDeg, -MAX_ONE_DIR_FOV, MAX_ONE_DIR_FOV);
      return Degrees.of(newSetpointDeg);
    }).withName("Turret.TrackAprilTag");
  }

  public Command trackAnyAprilTag() {
    return setAngleDynamic(() -> {
      double currentDeg = turret.getAngle().in(Degrees);

      if (!hasTarget()) {
        return Degrees.of(currentDeg);
      }

      double tx = getTagTx();

      if (Math.abs(tx) < TAG_TX_TOLERANCE_DEG) {
        return Degrees.of(currentDeg);
      }

      double correctionDeg = MathUtil.clamp(tx, -MAX_TRACK_STEP_DEG, MAX_TRACK_STEP_DEG);
      double newSetpointDeg = currentDeg - correctionDeg;

      newSetpointDeg = MathUtil.clamp(newSetpointDeg, -MAX_ONE_DIR_FOV, MAX_ONE_DIR_FOV);
      return Degrees.of(newSetpointDeg);
    }).withName("Turret.TrackAnyAprilTag");
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();

    Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        new Pose3d(
            turretTranslation,
            new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    });
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}

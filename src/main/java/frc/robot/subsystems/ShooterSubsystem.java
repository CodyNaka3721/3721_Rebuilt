package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

import com.ctre.phoenix6.hardware.TalonFX;



public class ShooterSubsystem extends SubsystemBase {
  // 2 Neos, 4in shooter wheels
  // private final ThriftyNova leaderNova = new ThriftyNova(
  // Constants.ShooterConstants.kLeaderMotorId,
  // ThriftyNova.MotorType.NEO);

  // private final ThriftyNova followerNova = new ThriftyNova(
  // Constants.ShooterConstants.kFollowerMotorId,
  // ThriftyNova.MotorType.NEO);

  private final TalonFX leaderTalon = new TalonFX(Constants.ShooterConstants.kLeaderMotorId);
  private final TalonFX followerTalon = new TalonFX(Constants.ShooterConstants.kFollowerMotorId);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerTalon, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00936, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(30));

  private final SmartMotorController smc = new TalonFXWrapper(leaderTalon, DCMotor.getKrakenX60(2), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);

  public ShooterSubsystem() {
    // leaderNova.factoryReset();
    // followerNova.factoryReset();

    // leaderNova.setVoltageCompensation(12);
    // followerNova.setVoltageCompensation(12);

    // leaderNova.setInverted(false);
    // followerNova.setInverted(true);

    // followerNova
    // .setInversion(true)
    // .follow(leaderNova.getID());
  }

  public Command setSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return shooter.setSpeed(speedSupplier);
  }

  public Command spinUp() {
    return Commands.runOnce(() -> smc.setVelocity(RPM.of(3000)), this).withName("Shooter.SpinUp");
    //return setSpeed(RPM.of(3000));

    // return setSpeed(RotationsPerSecond.of(50));

    // return run(() -> {
    // // followerNova.follow(leaderNova.getID());
    // // followerNova.setInverted(true);

    // // leaderNova.setPercent(SHOOTER_SPEED);
    // // followerNova.setPercent(SHOOTER_SPEED);

    // // followerNova.setPercent(0.5);
    // });

    // return shooter.set(0.5);
    // return shooter.setSpeed(RotationsPerSecond.of(500));
  }


  public Command PassShoot(){
    return Commands.runOnce(() -> smc.setVelocity(RPM.of(4500)), this).withName("Shooter.Pass");
  }
  public Command stopShootingCommand() {
    return Commands.runOnce(() -> smc.setVelocity(RPM.of(0)), this).withName("Shooter.Stop"); 
    // return run(() -> {

    // // leaderNova.setPercent(0);
    // // followerNova.setPercent(0);
    // // followerNova.setPercent(0.5);
    // });
    // return shooter.set(0);
  }

  public Command idleShootCommand() {
    return Commands.runOnce(() -> smc.setVelocity(RPM.of(2000)), this).withName("Shooter.IdleSpeed"); 
  }
  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  // public Command set(double dutyCycle) {
  // return shooter.set(dutyCycle);
  // }

  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }

  @Override
  public void periodic() {
    //Logger.recordOutput("Shooter/LeaderVelocity", leaderTalon.getVelocity());
    //Logger.recordOutput("Shooter/FollowerVelocity", followerTalon.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    // Calculate tangential velocity at the edge of the wheel and convert to
    // LinearVelocity

    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}




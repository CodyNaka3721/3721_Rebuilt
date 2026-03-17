// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser; 
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final TurretSubsystem turret = new TurretSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();  
    private final HopperSubsystem hopper = new HopperSubsystem();
    private final KickerSubsystem kicker= new KickerSubsystem();

    private final Superstructure superstructure = new Superstructure(shooter, turret, intake, hopper, kicker);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick board = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        autoChooser = null;

        /*autoChooser = AutoBuilder.buildAutoChooser();   //Setup for pathplanner auto maker thingy 
        SmartDashboard.putData("Auto Mode", autoChooser);
        //Append commands into pathplanner to use as eventmarkers 
        //Use form: NamedCommands.registerCommand("Name", command)
        NamedCommands.registerCommand("PivotDown", superstructure.setIntakePivotAngle(Degrees.of(107)));    //Registered pivot down command
        autoChooser.setDefaultOption("Do Nothing", null);*/

        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    //change if backward
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
           // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
       // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        /*
         *Joystick Buttons  
         */
        //While pressing button A, spin shooter up to speed 
        joystick.a().onTrue(superstructure.shootCommand());
        joystick.a().onFalse(superstructure.stopShootCommand());

        //When button B pressed, feed balls into shooter 
        joystick.b().onTrue(superstructure.kickerFeedCommand().alongWith(superstructure.hopperFeedCommand()));
        joystick.b().onFalse(superstructure.kickerStopCommand().alongWith(superstructure.hopper.stopCommand()));

        //When button x pressed, intake balls 
        joystick.x().onTrue(superstructure.intakeCommand());
        joystick.x().onFalse(superstructure.intakeStop());
        
        /*
         * **************
         * Board Buttons
         * ************** 
        */
        //Shooter on 1
        board.button(1).onTrue(superstructure.shootCommand());
        board.button(1).onFalse(superstructure.stopShootCommand());
        //Hooper+Kicker on 2
        board.button(2).onTrue(superstructure.kickerFeedCommand().alongWith(superstructure.hopperFeedCommand()));
        board.button(2).onFalse(superstructure.kickerStopCommand().alongWith(superstructure.hopper.stopCommand()));
        //Intake on 10
        board.button(10).onTrue(superstructure.intakeCommand());
        board.button(10).onFalse(superstructure.intakeStop());
        //Pivot down on 11+Pivot up on13
        board.button(11).onTrue(superstructure.setIntakePivotAngle(Degrees.of(107)));
        board.button(13).onTrue(superstructure.setIntakePivotAngle(Degrees.of(1)));
        board.button(4).onTrue(superstructure.setIntakePivotAngle(Degrees.of(80)));
        //Outtake on 12
        board.button(12).onTrue(superstructure.ejectCommand());
        board.button(12).onFalse(superstructure.ejectCommand());
        
        board.button(3).onTrue(superstructure.backFeedAllCommand());
        board.button(3).onFalse(superstructure.backFeedAllCommand());
  

        //joystick.leftBumper().onTrue(superstructure.AprilTrack());
        joystick.leftBumper().onTrue(turret.trackAprilTag(9,10).until(() -> Math.abs(turret.getTagTx()) < 5.0));
        
        joystick.rightBumper().onTrue(turret.center());
        
        SmartDashboard.putNumber("limelight tx", turret.getTagTx());
        //SmartDashboard.putNumber("Intake Arm Angle", intake.getIntakeAngle());

        /*
         * 107 degrees is the max to deploy the pivot
         * 0 is the pivot back in 
         */
       // joystick.povUp().onTrue(superstructure.setIntakePivotAngle(Degrees.of(107)));
    }
    public Command getAutonomousCommand() {
        return null; //autoChooser.getSelected();
    }
}

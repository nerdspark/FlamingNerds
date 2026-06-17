// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final SlewRateLimiter limiter = new SlewRateLimiter(5.0, -9999.0, 0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10.0, -9999.0, 0);
    private double lastTranslationMag = 0.0;
    private double lastRotMag = 0.0;

    private double MaxSpeed = 0.4 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.05)
        .withRotationalDeadband(MaxAngularRate * 0.05);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final XboxController driver = new XboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureDefault();
        configureBindings();
        // configureSysID();
    }

    private void configureBindings() {
        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    private void configureDefault() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                Translation2d inputTranslation = new Translation2d(-driver.getRightY() * MaxSpeed, -driver.getRightX() * MaxSpeed);
                double inputRot = driver.getLeftX() * MaxAngularRate;

                double inputMag = inputTranslation.getNorm();
                double inputRotMag = Math.abs(inputRot);

                double filteredMag;
                double filteredRotMag;

                if (inputMag < lastTranslationMag) {
                    filteredMag = inputMag;
                    limiter.reset(filteredMag);
                } else {
                    filteredMag = limiter.calculate(inputMag);
                }
                
                lastTranslationMag = filteredMag;

                if (inputRotMag < lastRotMag) {
                    filteredRotMag = inputRotMag;
                    rotLimiter.reset(filteredRotMag);
                } else {
                    filteredRotMag = rotLimiter.calculate(inputRotMag);
                }

                lastRotMag = filteredRotMag;

                Translation2d filteredTranslation;
                if (inputMag > 1e-6) {
                    filteredTranslation = inputTranslation.times(filteredMag / inputMag);
                } else {
                    filteredTranslation = new Translation2d();
                }

                double filteredRot;
                if (inputRotMag > 1e-6) {
                    filteredRot = inputRot * (filteredRotMag / inputRotMag);
                } else {
                    filteredRot = 0.0;
                }

                return drive
                    .withVelocityX(filteredTranslation.getX())
                    .withVelocityY(filteredTranslation.getY())
                    .withRotationalRate(filteredRot);
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // private void configureSysID() {
    //     // Run SysId routines when holding back/start and X/Y.
    //     // Note that each routine should be run exactly once in a single log.
    //     driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //     driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //     driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //     driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}

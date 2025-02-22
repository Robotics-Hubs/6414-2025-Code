// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHolder;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm.ArmPosition;

public class RobotContainer {
    private double MaxSpeed = 3.5;
        //TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
        // RotationsPerSecond.of(1.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick pilot = new CommandJoystick(1);
    private final CommandXboxController operator = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final CoralHolder coralHolder = new CoralHolder();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-pilot.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        pilot.trigger().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        operator.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        elevator.setDefaultCommand(elevator.runSetpointUntilReached(Meters.zero()).andThen(Commands.waitUntil(() -> false)));

        arm.setDefaultCommand(
        arm.moveToAndStayAtPosition(Arm.ArmPosition.IDLE));
        coralHolder.setDefaultCommand(coralHolder.runVoltage(0));
        operator.y().onTrue(Commands.sequence(
            arm.moveToPosition(ArmPosition.RUN_UP),
            elevator.runSetpointUntilReached(Meters.of(1.5)),//mag fixed
            arm.moveToAndStayAtPosition(ArmPosition.SCORE_L4)
                .alongWith(elevator.runSetpoint(Meters.of(1.5)))));
                // .beforeStarting(() -> coralHolder.prepareToScoreL4().schedule())));
        operator.a().onTrue(Commands.sequence(
            arm.moveToPosition(ArmPosition.RUN_UP),
            elevator.runSetpoint(Centimeters.of(3)),
            arm.moveToPosition(ArmPosition.IDLE)));
        operator.b().onTrue(Commands.sequence(
            arm.moveToPosition(ArmPosition.IDLE),
            elevator.runSetpoint(Meters.of(0.15)),//mag fixed
            arm.moveToAndStayAtPosition(ArmPosition.SCORE)
                .alongWith(elevator.runSetpoint(Meters.of(0.65)))));
        operator.leftTrigger(0.5).whileTrue(coralHolder.intakeCoralSequence().raceWith(
            arm.moveToAndStayAtPosition(Arm.ArmPosition.INTAKE),
            elevator.runSetpoint(Centimeters.of(3))));
        operator.rightTrigger(0.5).whileTrue(coralHolder.scoreCoral());
        // operator.leftBumper().onTrue(Commands.sequence(
        //     elevator.elevatorTalon1.set(0.1),
        //     elevator.elevatorTalon2.set(-0.1)));
        // operator.rightBumper().onTrue(Commands.sequence(
        //    elevator.elevatorTalon1.set(-0.1),
        //    elevator.elevatorTalon2.set(0.1)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

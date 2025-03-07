// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.Auto;
import frc.robot.commands.ReefAlignment;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHolder;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.subsystems.vision.apriltags.AprilTagVisionIOReal;
import frc.robot.subsystems.vision.apriltags.ApriltagVisionIOSim;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;
import frc.robot.utils.FieldMirroringUtils;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;
import java.util.function.Supplier;

public class RobotContainer {

    private final LoggedPowerDistribution powerDistribution;

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

    private final SendableChooser<Command> autoChooser;

    // Subsystems
    public final AprilTagVision aprilTagVision;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final CoralHolder coralHolder = new CoralHolder();

    public RobotContainer() {

        powerDistribution = LoggedPowerDistribution.getInstance(0, ModuleType.kRev);

        final List<PhotonCameraProperties> camerasProperties =
                VisionConstants.photonVisionCameras; // load configs stored directly in VisionConstants.java
        aprilTagVision = new AprilTagVision(new AprilTagVisionIOReal(camerasProperties), camerasProperties);

        // set Starting pose
        resetFieldAndOdometryForAuto(getStartingPoseAtBlueAlliance());

        //Add autonomousCommand
        configureAutoCommands();

        configureBindings();

        drivetrain.configureAuto();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        pilot.trigger().whileTrue(drivetrain.applyRequest(() -> brake));

        //hold algae
        pilot.button(2).onTrue(Commands.sequence(
                arm.moveToPosition(ArmPosition.RUN_UP)
        ));

        //set coral score assignment
        pilot.button(3).onTrue(Commands.sequence(
                arm.setAssignmentCoral()
        ));

        //set algae score assignment
        pilot.button(4).onTrue(Commands.sequence(
                arm.setAssignmentAlgae()
        ));

        //score position L4 up
        pilot.button(8).onTrue(Commands.sequence(
                coralHolder.prepareToScoreL4()
        ));
//
//        /* auto alignment left side*/
//        pilot.button(9).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision, 0,false, Commands::none));
//
//        /* auto alignment right side*/
//        pilot.button(10).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision,  0,true, Commands::none));
//
//        /* auto alignment left side*/
//        pilot.button(7).and(pilot.button(9)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision, 1,false, Commands::none));
//
//        /* auto alignment right side*/
//        pilot.button(7).and(pilot.button(10)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision,  1,true, Commands::none));
//
//        /* auto alignment left side*/
//        pilot.button(6).and(pilot.button(9)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision, 2,false, Commands::none));
//
//        /* auto alignment right side*/
//        pilot.button(6).and(pilot.button(10)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision,  2,true, Commands::none));
//
//        /* auto alignment left side*/
//        pilot.button(5).and(pilot.button(9)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision, 3,false, Commands::none));
//
//        /* auto alignment right side*/
//        pilot.button(5).and(pilot.button(10)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision,  3,true, Commands::none));
//
//        /* auto alignment left side*/
//        pilot.button(11).and(pilot.button(9)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision, 4,false, Commands::none));
//
//        /* auto alignment right side*/
//        pilot.button(11).and(pilot.button(10)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision,  4,true, Commands::none));
//
//        /* auto alignment left side*/
//        pilot.button(12).and(pilot.button(9)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision, 5,false, Commands::none));
//
//        /* auto alignment right side*/
//        pilot.button(12).and(pilot.button(10)).whileTrue(ReefAlignment.alignmentToBranch(
//                drivetrain, aprilTagVision,  5,true, Commands::none));

        // reset the field-centric heading on left bumper press
        operator.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        /*
          can not set default command, because clash with drive up and down, instead with button back + axis Y
          elevator.setDefaultCommand(elevator.applyRequest(() -> -operator.getLeftY()).andThen(Commands.waitUntil(() -> false)));
          elevator.setDefaultCommand(elevator.runSetpointUntilReached(Meters.zero()).andThen(Commands.waitUntil(() -> false)));
        */

        arm.setDefaultCommand(
                arm.moveToAndStayAtPosition(Arm.ArmPosition.IDLE).onlyIf(() -> arm.getCurrentAssignment() == Arm.Assignment.CORAL && elevator.getCurrentHeight().in(Meter) <= 0.19));
        coralHolder.setDefaultCommand(coralHolder.runVoltage(0));

        //IDLE position L1
        operator.a().onTrue(Commands.sequence(
                arm.moveToPosition(ArmPosition.RUN_UP).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.9),
                elevator.runSetpointUntilReached(Meters.of(0.29)).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.39),
                arm.moveToAndStayAtPosition(ArmPosition.IDLE).alongWith(elevator.runSetpoint(Centimeters.of(3)))
        ));

        //score position L2
        operator.x().onTrue(Commands.sequence(
                arm.moveToPosition(ArmPosition.RUN_UP).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 1.0),
                arm.moveToPosition(ArmPosition.SCORE).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.39 && elevator.getCurrentHeight().in(Meter) <= 0.39),    //score position L2
                arm.moveToAndStayAtPosition(ArmPosition.SCORE)
                        .alongWith(elevator.runSetpointUntilReached(Meters.of(0.168)).andThen(elevator.runSetpoint(Meters.of(0.16))))
        ));
        //score position L3
        operator.y().onTrue(Commands.sequence(
                arm.moveToPosition(ArmPosition.RUN_UP).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.39),
                arm.moveToPosition(ArmPosition.SCORE).onlyIf(() -> elevator.getCurrentHeight().in(Meter) <= 0.39),
                arm.moveToAndStayAtPosition(ArmPosition.SCORE)
                        .alongWith(elevator.runSetpointUntilReached(Meters.of(0.55)).andThen(elevator.runSetpoint(Meters.of(0.54))))
        ));

        //score position L4
        operator.b().onTrue(Commands.sequence(
                arm.moveToAndStayAtPosition(ArmPosition.SCORE_L4)
                        .alongWith(elevator.runSetpointUntilReached(Meters.of(1.118)).andThen(elevator.runSetpoint(Meters.of(1.111))))
        ));

        //intake
        operator.leftTrigger(0.5).whileTrue(Commands.sequence(
                elevator.runSetpointUntilReached(Meters.of(0.29)).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.39),
                coralHolder.intakeCoralSequence().raceWith(
                arm.moveToAndStayAtPosition(Arm.ArmPosition.INTAKE),
                elevator.runSetpoint(Centimeters.of(0)))
        ));

        //score
        operator.rightTrigger(0.5).whileTrue(coralHolder.scoreCoral());

        //pick up algae
        operator.leftBumper().onTrue(Commands.sequence(
                arm.moveToPosition(ArmPosition.RUN_UP).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.9),
                elevator.runSetpointUntilReached(Meters.of(0.39)).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.39),
                arm.moveToAndStayAtPosition(ArmPosition.PICK_UP).alongWith(elevator.runSetpoint(Meters.of(0.19)))
        ));

        //push algae
        operator.rightBumper().onTrue(Commands.sequence(
                arm.moveToPosition(ArmPosition.RUN_UP).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.9),
                elevator.runSetpointUntilReached(Meters.of(0.39)).onlyIf(() -> elevator.getCurrentHeight().in(Meter) > 0.39),
                arm.moveToPosition(ArmPosition.PUSH).alongWith(elevator.runSetpoint(Centimeters.of(3)))
        ));

        //keep press back button and push Y axis to holder algae
        operator.back().whileTrue(arm.moveToAndStayAtPosition(ArmPosition.PICK_UP).alongWith(elevator.applyRequest(() -> -operator.getLeftY())));
    }

    public Pose2d getStartingPoseAtBlueAlliance() {
        Pose2d poseAtLeft = new Pose2d(7.3, 6.15, Rotation2d.fromDegrees(180));
        return Auto.getRightSide() ? Auto.flipLeftRight(poseAtLeft) : poseAtLeft;
    }

    private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
        final Pose2d startingPose = FieldMirroringUtils.toCurrentAlliancePose(robotStartingPoseAtBlueAlliance);

        aprilTagVision
                .focusOnTarget(-1, -1)
                .withTimeout(0.1)
                .alongWith(Commands.runOnce(() -> drivetrain.setPose(startingPose), drivetrain))
                .ignoringDisable(true)
                .schedule();
    }

    private void configureAutoCommands() {
        NamedCommands.registerCommand("Aim At Score", arm.moveToPosition(ArmPosition.SCORE));
        NamedCommands.registerCommand("Run Idle", arm.moveToPosition(ArmPosition.IDLE));
        NamedCommands.registerCommand("Raise Up", elevator.runSetpointUntilReached(Meters.of(0.19)));
        NamedCommands.registerCommand("Coral Score", coralHolder.scoreCoral());
        NamedCommands.registerCommand("Raise Aim Score", elevator.runSetpointUntilReached(Meters.of(0.19))
                .deadlineFor(Commands.waitSeconds(0.1))
                .andThen(arm.moveToPosition(ArmPosition.SCORE))
                .andThen(coralHolder.scoreCoral()).asProxy());
    }

    public Command getAutonomousCommand() {
        /* This method loads the auto when it is called, however, it is recommended
        to first load your paths/autos when code starts, then return the
        preloaded auto/path */
        boolean isMirror = drivetrain.getCurrentAllianceColor() == DriverStation.Alliance.Red;
        String auto = "New Auto2";
        Commands.print(auto).schedule();
        return new PathPlannerAuto(auto, isMirror);

//        String path = "New Path";
//        Commands.print(path).schedule();
//        return drivetrain.getPathPlannerCommandFromPathFile(path);

//        return autoChooser.getSelected();
//        return Commands.print("No autonomous command configured");
    }
}
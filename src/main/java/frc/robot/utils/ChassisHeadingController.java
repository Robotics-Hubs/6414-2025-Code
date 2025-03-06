package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.constants.DriveControlLoops;
import frc.robot.utils.RobotPIDController;
import frc.robot.utils.RobotCommonMath;
import org.littletonrobotics.junction.Logger;

import java.util.OptionalDouble;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.constants.DriveControlLoops.ANGULAR_ACCELERATION_SOFT_CONSTRAIN;
import static frc.robot.constants.DriveControlLoops.ANGULAR_VELOCITY_SOFT_CONSTRAIN;

/**
 *
 *
 * <h1>Custom Controller for Chassis Heading</h1>
 */
public class ChassisHeadingController {

    /**
     *
     *
     * <h2>Represents an abstract chassis heading request.</h2>
     */
    public abstract static class ChassisHeadingRequest {}

    /**
     *
     *
     * <h2>Represents a request to face a specific rotation.</h2>
     *
     * <p>The chassis is instructed to rotate to and maintain a specified rotation.
     */
    public static class FaceToRotationRequest extends ChassisHeadingRequest {
        public final Rotation2d rotationTarget;

        /** @param rotationTarget the target rotation for the chassis */
        public FaceToRotationRequest(Rotation2d rotationTarget) {
            this.rotationTarget = rotationTarget;
        }
    }

    /**
     *
     *
     * <h2>Represents a request to face a target location on the field.</h2>
     *
     * <p>The chassis is instructed to aim toward a specified target on the field. Optionally, a
     * {@link RobotShooterOptimization} object can be provided to calculate time-of-flight for shooting-on-the-move
     * functionality.
     */
    public static class FaceToTargetRequest extends ChassisHeadingRequest {
        public final Supplier<Translation2d> target;
        public final RobotShooterOptimization shooterOptimization;

        /**
         * @param target the supplier providing the target position on the field
         * @param shooterOptimization optional; used to calculate shooter time-of-flight for shooting-on-the-move
         *     functionality; if not used, pass null
         */
        public FaceToTargetRequest(Supplier<Translation2d> target, RobotShooterOptimization shooterOptimization) {
            this.target = target;
            this.shooterOptimization = shooterOptimization;
        }
    }

    /**
     *
     *
     * <h2>Represents an empty request.</h2>
     *
     * <p>This request will cause to return zero correction speeds.
     */
    public static class NullRequest extends ChassisHeadingRequest {}

    private final TrapezoidProfile chassisRotationProfile;
    private final RobotPIDController chassisRotationCloseLoop;
    private final double maxAngularVelocityRadPerSec;
    private ChassisHeadingRequest headingRequest;
    private TrapezoidProfile.State chassisRotationState;

    /**
     *
     *
     * <h2>Constructs a heading controller with specific configurations.</h2>
     *
     * @param chassisRotationConstraints defines the maximum angular velocity and acceleration for the drivetrain
     * @param chassisRotationCloseLoopConfig PID configuration for chassis rotation
     * @param chassisInitialFacing the initial orientation of the robot
     */
    public ChassisHeadingController(
            TrapezoidProfile.Constraints chassisRotationConstraints,
            RobotPIDController.RobotPIDConfig chassisRotationCloseLoopConfig,
            Rotation2d chassisInitialFacing) {
        this.chassisRotationProfile = new TrapezoidProfile(chassisRotationConstraints);
        this.chassisRotationCloseLoop = new RobotPIDController(chassisRotationCloseLoopConfig);
        this.headingRequest = new NullRequest();
        this.maxAngularVelocityRadPerSec = chassisRotationConstraints.maxVelocity;
        this.chassisRotationState = new TrapezoidProfile.State(chassisInitialFacing.getRadians(), 0);
    }

    /**
     * Sets a new heading request.
     *
     * @param newRequest the new heading request
     */
    public void setHeadingRequest(ChassisHeadingRequest newRequest) {
        this.headingRequest = newRequest;
    }



    /**
     *
     *
     * <h2>Calculates rotational correction speeds for a face-to-rotation request.</h2>
     */
    private double calculateFaceToRotation(
            Pose2d robotPose, Rotation2d targetedRotation, double desiredAngularVelocityRadPerSec) {
        TrapezoidProfile.State goalState = getGoalState(targetedRotation);
        chassisRotationState =
                chassisRotationProfile.calculate(Robot.defaultPeriodSecs, chassisRotationState, goalState);

        final double feedBackSpeed =
                chassisRotationCloseLoop.calculate(robotPose.getRotation().getRadians(), chassisRotationState.position);
        final double feedForwardSpeedRadPerSec =
                Math.abs(targetedRotation.minus(robotPose.getRotation()).getDegrees()) < 15
                        ? desiredAngularVelocityRadPerSec
                        : chassisRotationState.velocity;

        log(robotPose, targetedRotation);

        return RobotCommonMath.constrainMagnitude(
                feedBackSpeed + feedForwardSpeedRadPerSec, maxAngularVelocityRadPerSec);
    }

    /**
     *
     *
     * <h2>Determines the closest goal for the targeted rotation.</h2>
     *
     * <p>Finds the closest rotational position on the profile that aligns with the target rotation. This ensures
     * continuity in the rotational profile.
     *
     * @param targetedRotation the desired orientation
     * @return a {@link TrapezoidProfile.State} representing the target goal state
     */
    private TrapezoidProfile.State getGoalState(Rotation2d targetedRotation) {
        final Rotation2d difference = targetedRotation.minus(Rotation2d.fromRadians(chassisRotationState.position));
        final double goal = chassisRotationState.position + difference.getRadians();
        return new TrapezoidProfile.State(goal, 0);
    }

    private boolean atSetPoint = false;

    private void log(Pose2d robotPose, Rotation2d requestedRotation) {
        Logger.recordOutput(
                "ChassisHeadingController/Requested", new Pose2d(robotPose.getTranslation(), requestedRotation));
        Logger.recordOutput(
                "ChassisHeadingController/CurrentState",
                new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(chassisRotationState.position)));
        final Rotation2d error = requestedRotation.minus(robotPose.getRotation());
        Logger.recordOutput("ChassisHeadingController/Error", error.getDegrees());
        atSetPoint = Math.abs(error.getRadians()) < chassisRotationCloseLoop.getErrorTolerance();
    }

    public boolean atSetPoint() {
        return atSetPoint;
    }

    private static ChassisHeadingController instance = null;

    public static ChassisHeadingController getInstance() {
        if (instance == null)
            instance = new ChassisHeadingController(
                    new TrapezoidProfile.Constraints(
                            ANGULAR_VELOCITY_SOFT_CONSTRAIN.in(RadiansPerSecond),
                            ANGULAR_ACCELERATION_SOFT_CONSTRAIN.in(RadiansPerSecondPerSecond)),
                    DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP,
                    new Rotation2d());

        return instance;
    }
}

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

/**
 * stores the constants and PID configs for chassis because we want an all-real simulation for the chassis, the numbers
 * are required to be considerably precise
 */
public class DriveTrainConstants {
    /** numbers that needs to be changed to fit each robot TODO: change these numbers to match your robot */
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.5;

    public static final Mass ROBOT_MASS = Kilograms.of(50); // robot weight with bumpers

    /** TODO: change motor type to match your robot */
    public static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getKrakenX60(1);

    /* adjust current limit */
    public static final Current DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT = Amps.of(80);

    /* for collision detection in simulation */
    public static final Distance BUMPER_WIDTH = Inches.of(33), BUMPER_LENGTH = Inches.of(30);

    // https://unacademy.com/content/upsc/study-material/physics/moment-of-inertia-of-rectangle-section/
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(ROBOT_MASS.in(Kilograms)
            * (BUMPER_WIDTH.in(Meters) * BUMPER_WIDTH.in(Meters) + BUMPER_LENGTH.in(Meters) * BUMPER_LENGTH.in(Meters))
            / 12.0);

    /** numbers imported from {@link TunerConstants} TODO: for REV chassis, replace them with actual numbers */
    public static final Distance WHEEL_RADIUS = Meters.of(TunerConstants.FrontLeft.WheelRadius);

    public static final double DRIVE_GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
    public static final double STEER_GEAR_RATIO = TunerConstants.FrontLeft.SteerMotorGearRatio;

    /* floor_speed = wheel_angular_velocity * wheel_radius */
    public static final LinearVelocity CHASSIS_MAX_VELOCITY = MetersPerSecond.of(DRIVE_MOTOR_MODEL.getSpeed(
            DRIVE_MOTOR_MODEL.getTorque(
                    DRIVE_MOTOR_MODEL.getCurrent(0, TunerConstants.FrontLeft.DriveFrictionVoltage)),
            12)
            / DRIVE_GEAR_RATIO
            * WHEEL_RADIUS.in(Meters));

    /** translations of the modules to the robot center, in FL, FR, BL, BR */
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
}

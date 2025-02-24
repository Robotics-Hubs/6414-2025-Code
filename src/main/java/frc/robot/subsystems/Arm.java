package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;


public class Arm extends SubsystemBase {
    private static final double ARM_GEAR_RATIO = 44.0/16.0 * 20.0;
    private static final Rotation2d armActualStartingPosition = new Rotation2d(-166.32, 203.20); // measured from cad

    private static final Angle lowerLimit = Degrees.of(30);
    private static final Angle upperLimit = Degrees.of(128);

    public enum ArmPosition {
        IDLE(116),
        INTAKE(125),
        SCORE(106),
        RUN_UP(55),
        SCORE_L4(70);

        private final Angle angle;
        ArmPosition(double degrees) {
            this.angle = Degrees.of(degrees);
        }
    }

    private final TalonFX armMotor = new TalonFX(41);
    private final StatusSignal<Current> armMotorCurrent;
    private final StatusSignal<Angle> armAngle;

    private static final Angle tolerance = Degrees.of(4);
    private final ArmFeedforward feedforwardController = new ArmFeedforward(0, 0.17, 0.8, 0.01);
    private final PIDController strongFeedBackController = new PIDController(9.0/Math.toRadians(30), 0, 0);
    private final PIDController weakFeedBackController = new PIDController(3.0/Math.toRadians(30), 0, 0);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Math.toRadians(360), Math.toRadians(480)));
    private TrapezoidProfile.State state;

    private Angle setpoint;
    public Arm() {
        armMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        armMotor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40));

        armAngle = armMotor.getPosition();

        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotorCurrent = armMotor.getSupplyCurrent();
       BaseStatusSignal.setUpdateFrequencyForAll(100.0,  armAngle, armMotorCurrent);

        this.state = new TrapezoidProfile.State(upperLimit.in(Radians), 0);
        this.setpoint = ArmPosition.IDLE.angle;
    }

    public Angle getArmAngle() {
        return Rotation2d.fromRotations(armAngle.getValueAsDouble() / ARM_GEAR_RATIO).plus(armActualStartingPosition).getMeasure();
    }

    public boolean atReference() {
        return getArmAngle().minus(setpoint).abs(Radians) < tolerance.in(Radians);
    }

    public void requestSetpoint(ArmPosition armPosition) {
        this.setpoint = armPosition.angle;
    }

    public Command moveToAndStayAtPosition(ArmPosition armPosition) {
        return run(() -> requestSetpoint(armPosition));
    }

    public Command moveToPosition(ArmPosition armPosition) {
        return moveToAndStayAtPosition(armPosition)
                .until(this::atReference);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(armMotorCurrent, armAngle);

        if (DriverStation.isEnabled())
            runController();
        else
            runIdle();

        SmartDashboard.putNumber("Arm/MotorCurrent (Amps)", armMotorCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Arm/Profile Angle (Deg)", Math.toDegrees(state.position));
        SmartDashboard.putNumber("Arm/Measured Angle (Deg)", getArmAngle().in(Degrees));
    }

    private double previousVelocity = 0;
    private void runController() {
        double currentPositionRad = getArmAngle().in(Radians);

        TrapezoidProfile.State goalState = new TrapezoidProfile.State(setpoint.in(Radians), 0);
        state = profile.calculate(Robot.kDefaultPeriod, state, goalState);

        double acceleration = (state.velocity - previousVelocity) / Robot.kDefaultPeriod;
        double feedforwardVoltage = feedforwardController.calculate(currentPositionRad, state.velocity, acceleration);
        PIDController feedBackController = Math.abs(state.velocity) < 0.03
                ? weakFeedBackController
                :strongFeedBackController;
        double feedbackVoltage = feedBackController.calculate(currentPositionRad, state.position);
        previousVelocity = state.velocity;
        setMotorVoltage(feedforwardVoltage + feedbackVoltage);
    }

    private void runIdle() {
        state = new TrapezoidProfile.State(getArmAngle().in(Radians), 0);
        previousVelocity = 0;

        setMotorVoltage(0);
    }

    private void setMotorVoltage(double volts) {
        if (getArmAngle().gt(upperLimit) && volts > 0)
            volts = 0;
        else if (getArmAngle().lt(lowerLimit) && volts < 0)
            volts = 0;
        volts = MathUtil.clamp(volts, -4, 4);
        armMotor.setControl(new VoltageOut(volts));
    }
}

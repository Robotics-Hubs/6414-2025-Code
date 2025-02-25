package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;

import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
    private static final Distance AT_REFERENCE_TOLERANCE = Centimeters.of(3);
    private static final Distance CHAIN_LENGTH = Inches.of(0.25);
    private static final int WHEEL_TEETH = 22;
    private static final double GEAR_RATIO = 84.0 / 12.0;
    private static final Distance LOWER_LIMIT = Meters.of(0.05);
    private static final Distance HEIGHT_UPPER_LIMIT = Meters.of(1.23);

    private static final Voltage minOutputVoltage = Volts.of(-2.0);
    private static final Voltage maxOutputVoltage = Volts.of(2.2);

    public final TalonFX elevatorTalon1 = new TalonFX(31);
    public final TalonFX elevatorTalon2 = new TalonFX(32);
    private final StatusSignal<Angle> elevatorMotorPosition;
    private final StatusSignal<Current> motor1SupplyCurrent;
    private final StatusSignal<Current> motor2SupplyCurrent;

    private final TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(4, 6));
    private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(0.1, 0.43, 1.11, 0.05, Robot.kDefaultPeriod);
    private final PIDController strongFeedBackController = new PIDController(10.0/0.2, 0, 0);
    private final PIDController weakFeedBackController = new PIDController(5.0/0.2, 0, 0);

    private Distance currentHeight;
    private TrapezoidProfile.State currentState;
    private Optional<Distance> setpoint;

    public Elevator() {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60);
        elevatorTalon1.getConfigurator().apply(currentLimitsConfigs);
        elevatorTalon2.getConfigurator().apply(currentLimitsConfigs);
        elevatorTalon1.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        elevatorTalon2.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        elevatorTalon1.setNeutralMode(NeutralModeValue.Brake);
        elevatorTalon2.setNeutralMode(NeutralModeValue.Brake);


        elevatorTalon1.setPosition(0);
        elevatorMotorPosition = elevatorTalon1.getPosition();
        motor1SupplyCurrent = elevatorTalon1.getSupplyCurrent();
        motor2SupplyCurrent = elevatorTalon2.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, elevatorMotorPosition, motor1SupplyCurrent, motor2SupplyCurrent);

        currentState = new TrapezoidProfile.State(0, 0);
        setpoint = Optional.empty();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(elevatorMotorPosition, motor1SupplyCurrent, motor2SupplyCurrent);
        double motorRotations = elevatorMotorPosition.getValue().in(Rotations);
        currentHeight = Meters.of(motorRotations / GEAR_RATIO * WHEEL_TEETH * CHAIN_LENGTH.in(Meters) * 2);

        setpoint.ifPresentOrElse(
                this::runControlLoops,
                this::runIdle);

        SmartDashboard.putNumber("Elevator Request Height (Meters)", setpoint.orElse(Meters.zero()).in(Meters));
        SmartDashboard.putNumber("Elevator Profile State (Meters)", currentState.position);
        SmartDashboard.putNumber("Elevator Filtered Height (Meters)", getCurrentHeight().in(Meters));
        SmartDashboard.putNumber("Elevator Motor Supply Current (Amps)", motor1SupplyCurrent.getValue().plus(motor2SupplyCurrent.getValue()).in(Amps));
    }

    private double previousVelocityMPS = 0;
    private void runControlLoops(Distance setpoint) {
        TrapezoidProfile.State goal = new TrapezoidProfile.State(setpoint.in(Meters), 0);
        currentState = profile.calculate(Robot.kDefaultPeriod, currentState, goal);

        double acceleration = (currentState.velocity - previousVelocityMPS) / Robot.kDefaultPeriod;
        double feedforwardVolts = feedforwardController.calculate(currentState.velocity);
//        double feedforwardVolts = feedforwardController.calculate(currentState.velocity, acceleration);
        previousVelocityMPS = currentState.velocity;
        PIDController feedBackController = Math.abs(currentState.velocity) < 0.04 ?
                weakFeedBackController
                :strongFeedBackController;
        double feedbackVolts = feedBackController.calculate(getCurrentHeight().in(Meters), currentState.position);

        runVoltage(Volts.of(feedforwardVolts + feedbackVolts));
        SmartDashboard.putNumber("Elevator FF Volts", feedforwardVolts);
        SmartDashboard.putNumber("Elevator FB Volts", feedbackVolts);
    }
    
    public void runIdle() {
        this.currentState = new TrapezoidProfile.State(getCurrentHeight().in(Meters), 0);
        this.previousVelocityMPS = 0;
        runVoltage(Volts.zero());
    }

    public void runVoltage(Voltage voltage) {
        if (isHigherLimitReached() && voltage.gt(Volts.zero()))
            voltage = Volts.zero();
        else if (isLowerLimitReached() && voltage.lt(Volts.zero()))
            voltage = Volts.zero();

        VoltageOut voltageOut = new VoltageOut(MathUtil.clamp(voltage.in(Volts), minOutputVoltage.in(Volts), maxOutputVoltage.in(Volts)));
        elevatorTalon1.setControl(voltageOut);
        elevatorTalon2.setControl(voltageOut);
    }
    public Command applyRequest(Supplier<Double> requestSupplier) {
        return run(() -> runVoltage(Volts.of(requestSupplier.get() * maxOutputVoltage.in(Volts))));
    }

    private boolean isLowerLimitReached() {
        return getCurrentHeight().lte(LOWER_LIMIT);
    }

    private boolean isHigherLimitReached() {
        return getCurrentHeight().gte(HEIGHT_UPPER_LIMIT);
    }

    public void requestSetpoint(Distance setpoint) {
        this.setpoint = Optional.of(setpoint);
    }

    public void cancelSetpoint() {
        this.setpoint = Optional.empty();
    }

    public Command runSetpoint(Distance setpoint) {
        return runEnd(
                () -> requestSetpoint(setpoint),
                this::cancelSetpoint);
    }

    public Command runSetpointUntilReached(Distance setpoint) {
        return runSetpoint(setpoint)
                .until(this::atReference);
    }

    public Distance getCurrentHeight() {
        return currentHeight;
    }

    public boolean atReference() {
        return setpoint.isPresent()
                && setpoint.get()
                .minus(getCurrentHeight())
                .abs(Meters) < AT_REFERENCE_TOLERANCE.abs(Meters);
    }
}

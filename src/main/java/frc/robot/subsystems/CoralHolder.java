package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralHolder extends SubsystemBase {
    private final LaserCan firstSensor = new LaserCan(0);
    private final LaserCan secondSensor = new LaserCan(1);
    private final TalonFX rollerTalon = new TalonFX(61);
    private final TalonFX intakeLeft = new TalonFX(1,"rio");
    private final TalonFX intakeRight = new TalonFX(2,"rio");
    private final StatusSignal<Current> rollerMotorCurrent;

    private boolean firstSensorTriggered = false;
    private boolean secondSensorTriggered = false;
    public CoralHolder() {
        rollerTalon.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20));
        rollerTalon.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive));
        this.rollerMotorCurrent = rollerTalon.getSupplyCurrent();
        this.rollerMotorCurrent.setUpdateFrequency(100.0);
        this.rollerTalon.setNeutralMode(NeutralModeValue.Brake);
    }

    private void setVoltage(double volts) {
        this.rollerTalon.setControl(new VoltageOut(volts));
        this.intakeLeft.setControl(new VoltageOut(volts));
        this.intakeRight.setControl(new VoltageOut(-volts));
    }

    @Override
    public void periodic() {
        firstSensorTriggered = isSensorTriggered(firstSensor);
        secondSensorTriggered = isSensorTriggered(secondSensor);

        SmartDashboard.putNumber("Intake/MotorCurrent (Amps)", rollerTalon.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/FirstSensorTriggered", firstSensorTriggered);
        SmartDashboard.putBoolean("Intake/SecondSensorTriggered", secondSensorTriggered);
    }

    private static boolean isSensorTriggered(LaserCan sensor) {
        LaserCan.Measurement measurement = sensor.getMeasurement();
        if (measurement == null)
            return false;
        if (measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
            return false;
        return measurement.distance_mm < 30;
    }

    public boolean hasCoral() {
        return firstSensorTriggered || secondSensorTriggered;
    }

    public boolean coralInPlace() {
        return firstSensorTriggered && secondSensorTriggered;
    }

    public Command runVoltage(double volts) { 
        return run(() -> setVoltage(volts));
    }

    public Command intakeCoralSequence() {
        return Commands.sequence(
                runVoltage(1.5).until(this::hasCoral),
                runVoltage(-0.5).withTimeout(0.1),
                runVoltage(0.6).until(this::coralInPlace))
                .finallyDo(() -> setVoltage(0.0))
                .onlyIf(() -> !this.coralInPlace());
    }

    public Command scoreCoral() {
        return runVoltage(1.5)
                .until(() -> !this.hasCoral())
                .finallyDo(() -> setVoltage(0.0));
    }

    public Command prepareToScoreL4() {
        return Commands.sequence(runVoltage(-0.8)
                .until(() -> !this.coralInPlace()),
                runVoltage(-0.5).withTimeout(0.6)
                .finallyDo(() -> setVoltage(0.0)));
    }
}

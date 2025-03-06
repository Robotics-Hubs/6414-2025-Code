package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class AlertsManager {
    private static final List<Alert> instances = new ArrayList<>();

    public static Alert create(String text, Alert.AlertType type) {
        Alert alert = new Alert(text, type);
        register(alert);
        return alert;
    }

    public static void register(Alert alert) {
        instances.add(alert);
    }

    private static boolean hasAlertType(Alert.AlertType type) {
        return instances.stream().filter(Alert::get).anyMatch(alert -> alert.getType() == type);
    }

    private static Command flashLEDForError = null;

    private static Command flashLEDForWarning = null;

    private static void runIfNotRunning(Command command) {
        if (!command.isScheduled()) command.schedule();
    }

    private static String getPrefix(Alert.AlertType type) {
        return switch (type) {
            case kError -> "Error: ";
            case kWarning -> "Warning: ";
            case kInfo -> "Info: ";
        };
    }
}

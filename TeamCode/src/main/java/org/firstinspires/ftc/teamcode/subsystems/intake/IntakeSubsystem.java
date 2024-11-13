package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;

public class IntakeSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final CRServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.servo = hardwareMap.get(CRServo.class, ServoMap.INTAKE.getId());
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.dataLogger.addData(DataLogger.DataType.INFO, "Initializing IntakeSubsystem.");
    }

    private void set(double power) {
        this.servo.setPower(power);
    }

    public Command setPower(double power) {
        return new StartEndCommand(() -> this.set(power), () -> this.setPower(0), this);
    }

    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }
}

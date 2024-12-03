package org.firstinspires.ftc.teamcode.subsystems.extender;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import top.symple.symplegraphdisplay.managers.data.DataListenerGroup;

public class ExtenderSubsystem extends SubsystemBase implements LoggerSubsystem, DataListenerGroup {
    private final SimpleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public ExtenderSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ExtenderSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.servo = new SimpleServo(hardwareMap, ServoMap.EXTENDER.getId(), 0, 300);
    }

    private void turnToAngle(double deg) {
        this.servo.turnToAngle(deg);
    }

    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public Command goToState(ExtenderState state) {
        return new InstantCommand(() -> this.turnToAngle(state.getDeg()), this);
    }

    public enum ExtenderState {
        IDLE(0),
        TAKE(0);

        private final double deg;

        ExtenderState(double deg) {
            this.deg = deg;
        }

        public double getDeg() {
            return deg;
        }
    }
}

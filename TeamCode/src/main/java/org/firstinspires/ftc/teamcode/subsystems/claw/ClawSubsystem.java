package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.SympleServo;

public class ClawSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public ClawSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.dataLogger.addData(DataLogger.DataType.INFO, "Initializing ClawSubsystem.");

        this.servo = new SympleServo(hardwareMap, ServoMap.CLAW.getId(), 0, 300);
    }

    private void setState(ClawState state) {
        this.servo.turnToAngle(state.getDeg());
    }

    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public Command moveToState(ClawState state) {
        return new InstantCommand(() -> this.setState(state), this);
    }

    public enum ClawState {
        OPEN(0),
        CLOSE(0);
        private double deg;

        ClawState(double deg) {
            this.deg = deg;
        }

        public double getDeg() {
            return deg;
        }
    }
}

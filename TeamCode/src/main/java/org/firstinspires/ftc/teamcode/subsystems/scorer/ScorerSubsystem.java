package org.firstinspires.ftc.teamcode.subsystems.scorer;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.SympleServo;

public class ScorerSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public ScorerSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.servo = new SympleServo(hardwareMap, ServoMap.SCORER.getId(), 0, 300);
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.dataLogger.addData(DataLogger.DataType.INFO, "Initializing ScorerSubsystem.");
    }

    private void setState(ScorerState state) {
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

    public Command moveToState(ScorerState state) {
        return new InstantCommand(() -> this.setState(state), this);
    }

    public enum ScorerState {
        SCORE(0),
        TAKE(0);

        private final double deg;

        ScorerState(double deg) {
            this.deg = deg;
        }

        public double getDeg() {
            return deg;
        }
    }
}

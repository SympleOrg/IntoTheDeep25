package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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

    private IntakeState currentState = IntakeState.IDLE;

    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing IntakeSubsystem.");

        this.servo = hardwareMap.get(CRServo.class, ServoMap.INTAKE.getId());
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;
    }

    private void set(double power) {
        this.servo.setPower(power);
    }

    public Command setState(IntakeState state) {
        return new StartEndCommand(() -> this.set(state.getPower()), () -> this.set(0), this);
    }

    public Command toggleState(IntakeState state) {
        return new ConditionalCommand(
                this.setState(state),
                this.setState(IntakeState.IDLE),
                () -> state != this.currentState
        );
    }


    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public enum IntakeState {
        TAKE(1),
        DROP(-1),
        IDLE(0);

        private final double power;

        IntakeState(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }
}

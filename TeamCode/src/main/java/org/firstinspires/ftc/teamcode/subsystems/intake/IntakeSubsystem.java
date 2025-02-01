package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.SympleServo;

public class IntakeSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private IntakeConstants.IntakeState currentState;

    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing IntakeSubsystem.");

        this.servo = new SympleServo(hardwareMap, ServoMap.INTAKE.getId(), 0, 300);
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;
    }

    private void setState(IntakeConstants.IntakeState state) {
        this.currentState = state;
        this.servo.turnToAngle(state.getDeg());
    }

    public Command goToState(IntakeConstants.IntakeState state) {
        this.currentState = state;
        return new InstantCommand(() -> this.setState(state), this);
    }

    public Command toggleStates(IntakeConstants.IntakeState state1, IntakeConstants.IntakeState state2) {
        return new ConditionalCommand(
                this.goToState(state1),
                this.goToState(state2),
                () -> state1 != this.currentState
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
}

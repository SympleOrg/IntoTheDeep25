package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;

public class IntakeSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final MotorEx motor;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private RobotConstants.IntakeConstants.IntakeState currentState = RobotConstants.IntakeConstants.IntakeState.IDLE;

    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing IntakeSubsystem.");

        this.motor = new MotorEx(hardwareMap, MotorMap.INTAKE.getId());
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;
    }

    private void moveMotor(double power) {
        this.motor.set(power);
    }

    public Command setState(RobotConstants.IntakeConstants.IntakeState state) {
        this.currentState = state;
        return new RunCommand(() -> this.moveMotor(state.getPower()), this)
                .whenFinished(() -> this.moveMotor(0));
    }

    public Command toggleState(RobotConstants.IntakeConstants.IntakeState state) {
        return new ConditionalCommand(
                this.setState(state),
                this.setState(RobotConstants.IntakeConstants.IntakeState.IDLE),
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
}

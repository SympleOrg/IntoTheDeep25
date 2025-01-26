package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.SympleServo;

public class ClawSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;
    private RobotConstants.ClawConstants.ClawState state = RobotConstants.ClawConstants.ClawState.CLOSE;

    public ClawSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ClawSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.servo = new SympleServo(hardwareMap, ServoMap.CLAW.getId(), 0, 300);
    }

    @Override
    public void periodic() {
        getTelemetry().addData("claw state",  this.getState().name());
    }

    private void setState(RobotConstants.ClawConstants.ClawState state) {
        this.state = state;
        this.servo.turnToAngle(state.getDeg());
    }

    public RobotConstants.ClawConstants.ClawState getState() {
        return state;
    }

    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public Command moveToState(RobotConstants.ClawConstants.ClawState state) {
        return new InstantCommand(() -> this.setState(state), this);
    }

    public Command toggleState() {
        return new ConditionalCommand(
                this.moveToState(RobotConstants.ClawConstants.ClawState.OPEN),
                this.moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                () -> this.state == RobotConstants.ClawConstants.ClawState.CLOSE
        );
    }
}

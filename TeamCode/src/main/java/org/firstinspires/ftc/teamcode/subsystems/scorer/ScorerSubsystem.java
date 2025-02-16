package org.firstinspires.ftc.teamcode.subsystems.scorer;

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

public class ScorerSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private RobotConstants.ScorerConstants.ScorerState state;

    public ScorerSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ScorerSubsystem.");

        this.servo = new SympleServo(hardwareMap, ServoMap.SCORER.getId(), 0, 300);
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;
    }

    private void setState(RobotConstants.ScorerConstants.ScorerState state) {
        this.state = state;
        this.servo.turnToAngle(state.getDeg());
    }

    public Command toggleState() {
        return new ConditionalCommand(
                this.moveToState(RobotConstants.ScorerConstants.ScorerState.TAKE),
                this.moveToState(RobotConstants.ScorerConstants.ScorerState.SCORE),
                () -> this.state == RobotConstants.ScorerConstants.ScorerState.SCORE
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

    public Command moveToState(RobotConstants.ScorerConstants.ScorerState state) {
        return new InstantCommand(() -> this.setState(state), this);
    }
}

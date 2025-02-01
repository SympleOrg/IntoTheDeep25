package org.firstinspires.ftc.teamcode.subsystems.intakejoint;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants.IntakeJointConstants;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.SympleServo;

public class IntakeXJointSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private IntakeJointConstants.JointXState state;

    public IntakeXJointSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing IntakeXJointSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.servo = new SympleServo(hardwareMap, ServoMap.INTAKE_X_JOINT.getId(), 0, 300);
    }

    private void setState(IntakeJointConstants.JointXState state) {
        this.state = state;
        this.servo.turnToAngle(state.getDeg());
    }

    public IntakeJointConstants.JointXState getState() {
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

    public Command moveToState(IntakeJointConstants.JointXState state) {
        return new InstantCommand(() -> this.setState(state), this);
    }

    public Command toggleStates(IntakeJointConstants.JointXState state1, IntakeJointConstants.JointXState state2) {
        return new ConditionalCommand(
                this.moveToState(state2),
                this.moveToState(state1),
                () -> this.state == state1
        );
    }
}

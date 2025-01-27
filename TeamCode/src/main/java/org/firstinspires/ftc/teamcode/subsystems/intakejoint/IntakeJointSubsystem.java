package org.firstinspires.ftc.teamcode.subsystems.intakejoint;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.SympleServo;

public class IntakeJointSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private RobotConstants.IntakeJointConstants.JointState state;

    public IntakeJointSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ClawSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.servo = new SympleServo(hardwareMap, ServoMap.INTAKE_JOINT.getId(), 0, 300);
    }

    private void setState(RobotConstants.IntakeJointConstants.JointState state) {
        this.state = state;
        this.servo.turnToAngle(state.getDeg());
    }

    public RobotConstants.IntakeJointConstants.JointState getState() {
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

    public Command moveToState(RobotConstants.IntakeJointConstants.JointState state) {
        return new InstantCommand(() -> this.setState(state), this);
    }
}

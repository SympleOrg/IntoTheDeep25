package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.SympleServo;

import java.util.Arrays;

public class ClawSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;
    private final RevColorSensorV3 colorSensor;

    private RobotConstants.ClawConstants.ClawState state = RobotConstants.ClawConstants.ClawState.CLOSE;

    public ClawSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ClawSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.servo = new SympleServo(hardwareMap, ServoMap.CLAW.getId(), 0, 300);
//        this.servo.enable();
        colorSensor = hardwareMap.get(RevColorSensorV3.class, SensorMap.CLAW_SENSOR.getId());
    }

    public boolean isSampleDetected() {
        return colorSensor.blue() >= 90 || colorSensor.red() >= 69;
    }


    @Override
    public void periodic() {
        getTelemetry().addData("claw state",  this.getState().name());
        getTelemetry().addData("isSampleDetected", isSampleDetected());
        getTelemetry().addData("Color Blue", colorSensor.blue());
        getTelemetry().addData("Color Red", colorSensor.red());
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

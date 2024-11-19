package org.firstinspires.ftc.teamcode.subsystems.extender;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class ExtenderSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final MotorEx motor;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public ExtenderSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ExtenderSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.motor = new MotorEx(hardwareMap, MotorMap.EXTENDER.getId());

        this.motor.resetEncoder();

        this.setDefaultCommand(this.holdPosition());
    }

    private void moveMotor(double power) {
        double ff = calcFeedForward();
        this.motor.set(power + ff);
    }

    private double calcFeedForward() {
        return Math.cos(Math.toRadians(this.getCurrentPosInDeg()));
    }

    private double getCurrentPosInDeg() {
        return MathUtil.countsToDeg(this.motor.getCurrentPosition(), MotorMap.EXTENDER.getTicksPerRev());
    }

    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public Command holdPosition() {
        return new RunCommand(() -> this.moveMotor(0));
    }

    public Command goTo(ExtenderState state) {
        PIDController pidController = new PIDController(0, 0, 0);

        return new FunctionalCommand(
                () -> {
                    pidController.reset();
                    pidController.setTolerance(1);
                    pidController.setSetPoint(state.getDeg());
                },
                () -> {
                    double power = pidController.calculate(this.getCurrentPosInDeg());

                    this.moveMotor(power);
                },
                (interrupted) -> {},
                pidController::atSetPoint,
                this
        );
    }

    public enum ExtenderState {
        PUT(0),
        TAKE(0);

        private final double deg;

        ExtenderState(double deg) {
            this.deg = deg;
        }

        public double getDeg() {
            return deg;
        }
    }
}

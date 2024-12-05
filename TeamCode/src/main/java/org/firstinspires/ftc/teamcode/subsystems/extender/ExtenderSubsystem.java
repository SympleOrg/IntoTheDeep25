package org.firstinspires.ftc.teamcode.subsystems.extender;

import static org.firstinspires.ftc.teamcode.subsystems.extender.ExternderConstants.Kd;
import static org.firstinspires.ftc.teamcode.subsystems.extender.ExternderConstants.Ki;
import static org.firstinspires.ftc.teamcode.subsystems.extender.ExternderConstants.Kp;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.maps.MotorMap;


import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import top.symple.symplegraphdisplay.managers.data.DataListenerGroup;

public class ExtenderSubsystem extends SubsystemBase implements LoggerSubsystem, DataListenerGroup {
    private final MotorEx motor;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public ExtenderSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ExtenderSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.motor = new MotorEx(hardwareMap, MotorMap.EXTENDER_MOTOR.getId());
        this.motor.resetEncoder();
    }

    private void setPower(double power) {
        this.motor.set(power);
    }

    private double getCurrentPosition() {
        return MathUtil.countsToDeg(this.motor.getCurrentPosition(), ExternderConstants.TICKS_PER_REV);
    }

    public Command goToState(ExternderConstants.ExtenderState state) {
        PIDController pidController = new PIDController(Kp, Ki, Kd);

        return new FunctionalCommand(
                // init
                () -> {
                    pidController.reset();
                    pidController.setTolerance(0);
                    pidController.setSetPoint(state.getDeg());
                },
                // execute
                () -> {
                    double power = pidController.calculate(this.getCurrentPosition());
                    this.setPower(power);
                },
                // end
                (interrupted) -> {},
                pidController::atSetPoint,
                this
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

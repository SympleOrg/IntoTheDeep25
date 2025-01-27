package org.firstinspires.ftc.teamcode.subsystems.extender;

import static org.firstinspires.ftc.teamcode.RobotConstants.ExtenderConstants.Kd;
import static org.firstinspires.ftc.teamcode.RobotConstants.ExtenderConstants.Ki;
import static org.firstinspires.ftc.teamcode.RobotConstants.ExtenderConstants.Kp;
import static org.firstinspires.ftc.teamcode.RobotConstants.ExtenderConstants.MAX_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.ExtenderConstants.MIN_POS;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;


import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;

import top.symple.symplegraphdisplay.managers.data.DataListenerGroup;

public class ExtenderSubsystem extends SubsystemBase implements LoggerSubsystem, DataListenerGroup {
    private final MotorEx motor;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public ExtenderSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ExtenderSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.motor = new MotorEx(hardwareMap, MotorMap.EXTENDER.getId());
        this.motor.setInverted(true);
        this.motor.resetEncoder();

        this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        this.telemetry.addData("extender dist", this.getCurrentPosition());
    }

    private void setPower(double power) {
        double finalPower = power;
        double currentPos = getCurrentPosition();
        if(currentPos >= MAX_POS && -power > 0) finalPower = 0;
        if(currentPos <= MIN_POS && -power < 0) finalPower = 0;
        this.motor.set(finalPower);
    }

    public Command goToRest() {
        PIDController pidController = new PIDController(Kp, Ki, Kd);

        return new FunctionalCommand(
                // init
                () -> {
                    pidController.reset();
                    pidController.setTolerance(0);
                    pidController.setSetPoint(0);
                },
                // execute
                () -> {
                    double power = pidController.calculate(this.getCurrentPosition());
                    this.setPower(-power);
                },
                // end
                (interrupted) -> {},
                pidController::atSetPoint,
                this
        );
    }

    private double getCurrentPosition() {
        return RobotConstants.ExtenderConstants.METER_PER_TICK * this.motor.getCurrentPosition();
    }

    public Command moveWithJoyStick(GamepadEx gamepadEx) {
        return new RunCommand(() -> {
            this.setPower(gamepadEx.getRightY());
        }, this);
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

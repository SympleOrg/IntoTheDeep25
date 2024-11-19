package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;

public class ElevatorSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final MotorGroup motors;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public ElevatorSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ElevatorSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        MotorEx rightMotor = new MotorEx(hardwareMap, MotorMap.ELEVATOR_RIGHT.getId());
        MotorEx leftMotor = new MotorEx(hardwareMap, MotorMap.ELEVATOR_LEFT.getId());

        rightMotor.setInverted(true);

        this.motors = new MotorGroup(leftMotor, rightMotor);

        this.motors.resetEncoder();

        this.setDefaultCommand(this.holdElevator());
    }

    private void setPower(double power) {
        this.motors.set(power + ElevatorConstants.KG);
    }

    /**
     * @return The current position in meters
     */
    private double getCurrentPosition() {
        double leaderMotorPosition = this.motors.getPositions().get(0);
        return leaderMotorPosition * ElevatorConstants.METERS_PER_TICK;
    }


    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public Command holdElevator() {
        return new RunCommand(() -> this.setPower(0));
    }

    public Command goToState(ElevatorState state) {
        PIDController pidController = new PIDController(0, 0, 0);

        return new FunctionalCommand(
                // init
                () -> {
                    pidController.reset();
                    pidController.setTolerance(0.03);
                    pidController.setSetPoint(state.getMeters());
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

    public enum ElevatorState {
        BASKET_TOP(0),
        BASKET_BOTTOM(0),
        HUMAN_PLAYER(0),
        SCORE_TOP(0),
        SCORE_BOTTOM(0),
        REST(0);

        private final double meters;

        ElevatorState(double meters) {
            this.meters = meters;
        }

        public double getMeters() {
            return meters;
        }
    }
}

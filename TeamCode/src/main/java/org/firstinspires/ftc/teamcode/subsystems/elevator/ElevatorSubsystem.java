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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;

public class ElevatorSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final MotorGroup motors;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private final TouchSensor touchSensor;

    public ElevatorSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        dataLogger.addData(DataLogger.DataType.INFO, "Initializing ElevatorSubsystem.");

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        MotorEx rightMotor = new MotorEx(hardwareMap, MotorMap.ELEVATOR_RIGHT.getId());
        MotorEx leftMotor = new MotorEx(hardwareMap, MotorMap.ELEVATOR_LEFT.getId());

        this.motors = new MotorGroup(leftMotor, rightMotor);
        this.motors.setInverted(true);

        this.motors.resetEncoder();

        this.touchSensor = hardwareMap.get(TouchSensor.class, SensorMap.ELEVATOR_RESET.getId());

        this.setDefaultCommand(this.holdElevator());
    }

    @Override
    public void periodic() {
        this.getTelemetry().addData("elev pos", this.getCurrentPosition());
        this.getTelemetry().addData("elev cmd", this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");
        this.getTelemetry().addData("elev on ground", this.touchSensor.isPressed());

        if (this.touchSensor.isPressed()) this.motors.resetEncoder();
    }

    private void setPower(double power) {
        double finalPower = 0;
        double pos = this.getCurrentPosition();

        if(ElevatorConstants.MIN_HEIGHT < pos && pos < ElevatorConstants.MAX_HEIGHT) {
            finalPower = power;
        }

        this.motors.set(finalPower + ElevatorConstants.KG);
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
        return new RunCommand(() -> this.setPower(0), this);
    }

    private Command goToPosition(double meters) {
        PIDController pidController = new PIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D);

        return new FunctionalCommand(
                // init
                () -> {
                    pidController.reset();
                    pidController.setTolerance(0.01);
                    pidController.setSetPoint(meters);
                },
                // execute
                () -> {
                    double power = pidController.calculate(this.getCurrentPosition());
                    this.setPower(power);
                },
                // end
                (interrupted) -> {

                },
                pidController::atSetPoint,
                this
        );
    }

    public Command goToState(ElevatorConstants.ElevatorState state) {
        return goToPosition(state.getMeters());
    }

    public Command scoreOnChamber(ElevatorConstants.ElevatorState state) {
        return goToPosition(state.getMeters() - 0.1);
    }
}

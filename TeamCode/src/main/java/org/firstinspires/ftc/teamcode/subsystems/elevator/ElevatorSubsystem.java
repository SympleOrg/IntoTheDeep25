package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
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
        MotorEx motor = new MotorEx(hardwareMap, MotorMap.ELEVATOR_RIGHT.getId());
        motor.setInverted(true);

        this.motors = new MotorGroup(
                new MotorEx(hardwareMap, MotorMap.ELEVATOR_LEFT.getId()),
                motor
        );
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.dataLogger.addData(DataLogger.DataType.INFO, "Initializing ElevatorSubsystem.");
    }

    private void setPower(double power) {
        this.motors.set(power + ElevatorConstants.KG);
    }

    @Override
    public DataLogger getDataLogger() {
        return dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return telemetry;
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

package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;

// THIS CODE IS OUTDATED AND DIDN'T GET TESTED ON THE NEW ROBOT!
public class TankDriveSubsystem extends SubsystemBase implements IDriveTrainSubsystem {
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private boolean invert = false;

    private final MotorGroup leftMotor, rightMotor;

    public TankDriveSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Getting motors");
//        this.rightMotor = new MotorEx(hardwareMap, "right_wheels");
//        this.leftMotor = new MotorEx(hardwareMap, "left_wheels");
        this.leftMotor = new MotorGroup(
                new MotorEx(hardwareMap, MotorMap.LEG_FRONT_LEFT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_BACK_LEFT.getId())
        );
        this.rightMotor = new MotorGroup(
                new MotorEx(hardwareMap, MotorMap.LEG_FRONT_RIGHT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_BACK_RIGHT.getId())
        );
        this.rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setInverted(true);
    }

    public void moveMotors(double leftPower, double rightPower) {
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    @Override
    public void moveSideMotors(double left, double right) {
        this.moveMotors(left, right);
    }

    @Override
    public double getForwardDistanceDriven() {
        return RobotPositionManager.getInstance().getRightWheelDistanceDriven();
    }

    @Override
    public double getHeading() {
        return RobotPositionManager.getInstance().getHeadingByGyro();
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return this.telemetry;
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }

    public void setInverted(boolean invert) {
        this.invert = invert;
    }

    public boolean isInverted() {
        return this.invert;
    }
}

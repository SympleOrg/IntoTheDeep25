package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.ThreeDeadWheelLocalizer;

import java.util.HashMap;

public class MecanumDriveSubsystem extends SubsystemBase implements IDriveTrainSubsystem {
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private final MecanumChassisWheelSet mecanumChassisWheelSet;
    private final ThreeDeadWheelLocalizer localizer;
    private final MecanumDriveKinematics mecanumDriveKinematics;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Getting motors");
        this.mecanumChassisWheelSet = new MecanumChassisWheelSet(
                new MotorEx(hardwareMap, MotorMap.LEG_FRONT_LEFT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_FRONT_RIGHT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_BACK_LEFT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_BACK_RIGHT.getId())
        );

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Inverting motors");
        this.mecanumChassisWheelSet.setInverted(MecanumChassisWheelSet.MotorNames.FRONT_RIGHT, true);
        this.mecanumChassisWheelSet.setInverted(MecanumChassisWheelSet.MotorNames.BACK_RIGHT, true);

        this.localizer = new ThreeDeadWheelLocalizer(
                this.mecanumChassisWheelSet.getMotor(MecanumChassisWheelSet.MotorNames.FRONT_LEFT), // left dead wheel
                this.mecanumChassisWheelSet.getMotor(MecanumChassisWheelSet.MotorNames.FRONT_RIGHT), // right dead wheel
                this.mecanumChassisWheelSet.getMotor(MecanumChassisWheelSet.MotorNames.BACK_LEFT), // back dead wheel
                new Pose2d(), // starting pose
                DriveConstants.DeedWheels.WHEEL_RADIUS,
                DriveConstants.DeedWheels.TICKS_PER_REV,
                DriveConstants.DeedWheels.WHEELS_DISTANCE,
                DriveConstants.DeedWheels.THIRD_WHEEL_OFFSET
        );

        this.mecanumDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(0.168, 0.22585),
                new Translation2d(0.168, -0.22585),
                new Translation2d(-0.168, 0.22585),
                new Translation2d(-0.168, -0.22585)
        );
    }

    public void moveMotor(MecanumChassisWheelSet.MotorNames motor, double power) {
        MotorEx m = this.mecanumChassisWheelSet.getMotor(motor);
        if (m == null) {
            this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": failed to set power to the motor '" + motor.name() + "'");
            return;
        }
        m.set(power);
    }

    public void moveFromWheelSpeed(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(chassisSpeeds);

        double maxSpeed = Math.max(
                Math.max(Math.abs(wheelSpeeds.frontLeftMetersPerSecond), Math.abs(wheelSpeeds.frontRightMetersPerSecond)),
                Math.max(Math.abs(wheelSpeeds.rearLeftMetersPerSecond), Math.abs(wheelSpeeds.rearRightMetersPerSecond))
        );

        if(maxSpeed > 1.0f) {
            wheelSpeeds.frontLeftMetersPerSecond /= maxSpeed;
            wheelSpeeds.frontRightMetersPerSecond /= maxSpeed;
            wheelSpeeds.rearLeftMetersPerSecond /= maxSpeed;
            wheelSpeeds.rearRightMetersPerSecond /= maxSpeed;
        }

        this.moveMotor(MecanumChassisWheelSet.MotorNames.FRONT_LEFT, wheelSpeeds.frontLeftMetersPerSecond);
        this.moveMotor(MecanumChassisWheelSet.MotorNames.FRONT_RIGHT, wheelSpeeds.frontRightMetersPerSecond);
        this.moveMotor(MecanumChassisWheelSet.MotorNames.BACK_LEFT, wheelSpeeds.rearLeftMetersPerSecond);
        this.moveMotor(MecanumChassisWheelSet.MotorNames.BACK_RIGHT, wheelSpeeds.rearRightMetersPerSecond);
    }

    @Override
    public void moveSideMotors(double left, double right) {
        this.moveMotor(MecanumChassisWheelSet.MotorNames.FRONT_LEFT, left);
        this.moveMotor(MecanumChassisWheelSet.MotorNames.BACK_LEFT, left);

        this.moveMotor(MecanumChassisWheelSet.MotorNames.FRONT_RIGHT, right);
        this.moveMotor(MecanumChassisWheelSet.MotorNames.BACK_RIGHT, right);
    }

    @Override
    public double getForwardDistanceDriven() {
        return RobotPositionManager.getInstance().getRightWheelDistanceDriven();
    }

    public double getSideDistanceDriven() {
        return RobotPositionManager.getInstance().getBackWheelDistanceDriven();
    }

    public ThreeDeadWheelLocalizer getLocalizer() {
        return this.localizer;
    }

    @Override
    public double getHeading() {
        return RobotPositionManager.getInstance().getHeadingByGyro();
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return this.telemetry;
    }

    @Override
    public void periodic() {
        this.localizer.update();
    }

    public static class MecanumChassisWheelSet {
        private final HashMap<MotorNames, MotorEx> motors = new HashMap<>();

        public MecanumChassisWheelSet(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight) {
            this.motors.put(MotorNames.FRONT_LEFT, frontLeft);
            this.motors.put(MotorNames.FRONT_RIGHT, frontRight);
            this.motors.put(MotorNames.BACK_LEFT, backLeft);
            this.motors.put(MotorNames.BACK_RIGHT, backRight);
        }

        public void setInverted(MotorNames motorName, boolean inverted) {
            this.motors.get(motorName).setInverted(inverted);
        }

        public void setPower(MotorNames motorName, double power) {
            this.motors.get(motorName).set(power);
        }

        public MotorEx getMotor(MotorNames motorName) {
            return this.motors.get(motorName);
        }

        public enum MotorNames {
            FRONT_RIGHT,
            FRONT_LEFT,
            BACK_LEFT,
            BACK_RIGHT
        }
    }
}

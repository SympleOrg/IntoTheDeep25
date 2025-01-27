package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.trajectories.AutoPath;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.ThreeDeadWheelLocalizer;

import java.util.HashMap;
import java.util.Map;

public class MecanumDriveSubsystem extends SubsystemBase implements IDriveTrainSubsystem, AutoableDriveTrain, LoggerSubsystem {
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;
    private final ThreeDeadWheelLocalizer localizer;

    private final MecanumChassisWheelsSet wheelsSet;

    private AutoPath autoPath;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Pose2d startngPose, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Creating chassis wheels set");
        this.wheelsSet = new MecanumChassisWheelsSet(
                new MotorEx(hardwareMap, MotorMap.LEG_FRONT_LEFT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_FRONT_RIGHT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_BACK_LEFT.getId()),
                new MotorEx(hardwareMap, MotorMap.LEG_BACK_RIGHT.getId())
        );

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Inverting motors");
        this.wheelsSet.setInverted(MecanumChassisWheelsSet.MotorNames.FRONT_RIGHT, true);
        this.wheelsSet.setInverted(MecanumChassisWheelsSet.MotorNames.BACK_RIGHT, true);

        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Creating localizer");
        this.localizer = new ThreeDeadWheelLocalizer(
            this.wheelsSet.getMotor(MecanumChassisWheelsSet.MotorNames.FRONT_LEFT),
            this.wheelsSet.getMotor(MecanumChassisWheelsSet.MotorNames.FRONT_RIGHT),
            this.wheelsSet.getMotor(MecanumChassisWheelsSet.MotorNames.BACK_LEFT),
            startngPose,
            DriveConstants.DeedWheels.WHEEL_RADIUS,
            DriveConstants.DeedWheels.TICKS_PER_REV,
            DriveConstants.DeedWheels.WHEELS_DISTANCE,
            DriveConstants.DeedWheels.THIRD_WHEEL_OFFSET
        );
        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Localizer Created");
    }

    @Override
    public void periodic() {
        this.localizer.updatePose();
        Pose2d pose2d = this.localizer.getPose();
        this.getTelemetry().addData("Pos X", pose2d.getX());
        this.getTelemetry().addData("Pos Y", pose2d.getY());
        this.getTelemetry().addData("Pos Heading", pose2d.getRotation().getDegrees());

        MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = this.wheelsSet.getWheelSpeeds();
        this.getTelemetry().addData("fl", mecanumDriveWheelSpeeds.frontLeftMetersPerSecond);
        this.getTelemetry().addData("fr", mecanumDriveWheelSpeeds.frontRightMetersPerSecond);
        this.getTelemetry().addData("bl", mecanumDriveWheelSpeeds.rearLeftMetersPerSecond);
        this.getTelemetry().addData("br", mecanumDriveWheelSpeeds.rearRightMetersPerSecond);

//        Rotation2d halfv = pose2d.getRotation().times(9 / 2f);
//        Pose2d pos1 = pose2d.plus(new Transform2d(new Translation2d(), halfv));
//        Pose2d pos2 = pos1.plus(new Transform2d(new Translation2d(), halfv));

        TelemetryPacket packet = new TelemetryPacket(true);
        Pose2d drawPose = new Pose2d(
                MathUtil.meterToInch(pose2d.getX()),
                MathUtil.meterToInch(pose2d.getY()),
                pose2d.getRotation()
        );
        double radius = MathUtil.meterToInch(DriveConstants.ROBOT_RADIUS);
        Canvas canvas = packet.fieldOverlay()
                .setStrokeWidth(1)
                .strokeCircle(drawPose.getX(), drawPose.getY(), radius)
                .strokeLine(drawPose.getX(), drawPose.getY(), drawPose.getX() + drawPose.getRotation().getCos() * radius,  drawPose.getY() + drawPose.getRotation().getSin() * radius);

        if(autoPath != null) {
            autoPath.drawPath(canvas);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void moveMotor(MecanumChassisWheelsSet.MotorNames motor, double power) {
        this.wheelsSet.setPower(motor, power);
    }

    @Override
    public void moveSideMotors(double left, double right) {
        this.wheelsSet.setPower(MecanumChassisWheelsSet.MotorNames.FRONT_LEFT, left);
        this.wheelsSet.setPower(MecanumChassisWheelsSet.MotorNames.BACK_LEFT, left);

        this.wheelsSet.setPower(MecanumChassisWheelsSet.MotorNames.FRONT_RIGHT, right);
        this.wheelsSet.setPower(MecanumChassisWheelsSet.MotorNames.BACK_RIGHT, right);
    }

    @Override
    public double getForwardDistanceDriven() {
        return RobotPositionManager.getInstance().getRightWheelDistanceDriven();
    }

    public double getSideDistanceDriven() {
        return RobotPositionManager.getInstance().getBackWheelDistanceDriven();
    }

    public void setPose(Pose2d pose) {
//        this.localizer.setPose(pose);
    }

    @Override
    public Pose2d getPosition() {
        return this.localizer.getPose();
    }

    @Override
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        MecanumDriveWheelSpeeds speeds = this.wheelsSet.getWheelSpeeds();

        return new DifferentialDriveWheelSpeeds(
                (speeds.frontLeftMetersPerSecond + speeds.rearLeftMetersPerSecond) / 2,
                (speeds.frontRightMetersPerSecond + speeds.rearRightMetersPerSecond) / 2
        );
    }

    @Override
    public double getHeading() {
        return RobotPositionManager.getInstance().getHeadingByGyro();
    }

    public AutoPath setAutoPath(AutoPath autoPath) {
        this.autoPath = autoPath;
        return this.autoPath;
    }

    public AutoPath getAutoPath() {
        return autoPath;
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return this.telemetry;
    }

    public Command followTrajectory(Trajectory trajectory) {
        return new FollowTrajectoryCommand(this, trajectory);
    }

    public static class MecanumChassisWheelsSet {
        private final HashMap<MotorNames, MotorEx> motors = new HashMap<>();

        public MecanumChassisWheelsSet(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight) {
            this.motors.put(MotorNames.FRONT_LEFT, frontLeft);
            this.motors.put(MotorNames.FRONT_RIGHT, frontRight);
            this.motors.put(MotorNames.BACK_LEFT, backLeft);
            this.motors.put(MotorNames.BACK_RIGHT, backRight);
        }

        public void setInverted(MotorNames motorName, boolean inverted) {
            this.motors.get(motorName).setInverted(inverted);
        }

        public void setPower(MotorNames motorName, double power) {

            this.motors.get(motorName).set(
                    motorName == MotorNames.BACK_LEFT
                    ? power + Math.signum(power) * DriveConstants.MOTOR_POWER_FIX
                    : power
            );
        }

        public MotorEx getMotor(MotorNames motorName) {
            return this.motors.get(motorName);
        }

        public MecanumDriveWheelSpeeds getWheelSpeeds() {
            return new MecanumDriveWheelSpeeds(
                    MathUtil.encoderTPSToMPS(
                            this.motors.get(MotorNames.FRONT_LEFT).getVelocity(),
                            DriveConstants.WHEEL_RADIUS, MotorMap.LEG_FRONT_LEFT.getTicksPerRev()
                    ),
                    MathUtil.encoderTPSToMPS(
                            this.motors.get(MotorNames.FRONT_RIGHT).getVelocity(),
                            DriveConstants.WHEEL_RADIUS, MotorMap.LEG_FRONT_RIGHT.getTicksPerRev()
                    ),
                    MathUtil.encoderTPSToMPS(
                            this.motors.get(MotorNames.BACK_LEFT).getVelocity(),
                            DriveConstants.WHEEL_RADIUS, MotorMap.LEG_BACK_LEFT.getTicksPerRev()
                    ),
                    MathUtil.encoderTPSToMPS(
                            this.motors.get(MotorNames.BACK_RIGHT).getVelocity(),
                            DriveConstants.WHEEL_RADIUS, MotorMap.LEG_BACK_RIGHT.getTicksPerRev()
                    )
            );
        }

        public enum MotorNames {
            FRONT_RIGHT,
            FRONT_LEFT,
            BACK_LEFT,
            BACK_RIGHT
        }
    }
}

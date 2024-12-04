package org.firstinspires.ftc.teamcode.managers;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class RobotPositionManager {
    private final BHI260IMU imu;
    private final MotorEx rightDeadWheel;
    private final MotorEx leftDeadWheel;
    private final MotorEx backDeadWheel;

    private final MecanumDriveOdometry odometry;

    private final double startingAngle;

    private static RobotPositionManager instance;

    private RobotPositionManager(HardwareMap hardwareMap) {
        BHI260IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIRECTION, DriveConstants.USB_FACING_DIRECTION));
        this.imu = hardwareMap.get(BHI260IMU.class, "imu");
        this.imu.initialize(parameters);

        this.rightDeadWheel = new MotorEx(hardwareMap, SensorMap.DEAD_WHEEL_RIGHT.getId());
        this.leftDeadWheel = new MotorEx(hardwareMap, SensorMap.DEAD_WHEEL_LEFT.getId());
        this.backDeadWheel = new MotorEx(hardwareMap, SensorMap.DEAD_WHEEL_BACK.getId());

        this.rightDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);
        this.leftDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);

        MecanumDriveKinematics mecanumDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(0.22585, 0.168),
                new Translation2d(-0.22585, 0.168),
                new Translation2d(0.22585, -0.168),
                new Translation2d(-0.22585, -0.168)
        );

        this.odometry = new MecanumDriveOdometry(
            mecanumDriveKinematics,
            getHeadingByGyro2d(),
            new Pose2d(0, 0, new Rotation2d())
        );


        this.rightDeadWheel.resetEncoder();
        this.leftDeadWheel.resetEncoder();
        this.backDeadWheel.resetEncoder();

        this.startingAngle = getHeadingByGyro();
    }

    public static void init(HardwareMap hardwareMap) {
        instance = new RobotPositionManager(hardwareMap);
    }

    public void update() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(

        );

        this.odometry.update();

    }

    public static RobotPositionManager getInstance() {
        return instance;
    }

    public double getHeadingByGyro() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public Rotation2d getHeadingByGyro2d() {
        return Rotation2d.fromDegrees(getHeadingByGyro());
    }

    public double getRelativeHeading() {
        return this.getHeadingByGyro() - this.startingAngle;
    }

    public double getHeadingByWheels() {
        double right = this.getRightWheelDistanceDriven();
        double left = this.getLeftWheelDistanceDriven();
        return Math.toDegrees((right - left) / DriveConstants.WHEELS_DISTANCE);
    }

    public double getLeftWheelDistanceDriven() {
        return MathUtil.encoderTicksToMeter(this.leftDeadWheel.getCurrentPosition());
    }

    public double getRightWheelDistanceDriven() {
        return MathUtil.encoderTicksToMeter(this.rightDeadWheel.getCurrentPosition());
    }

    public double getBackWheelDistanceDriven() {
        return MathUtil.encoderTicksToMeter(this.backDeadWheel.getCurrentPosition());
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        this.odometry.updateWithTime()updatePose(pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
                this.backDeadWheel.get() * DriveConstants.
        )
    }
}

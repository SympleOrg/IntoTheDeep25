package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import java.util.function.DoubleSupplier;

public class ThreeDeadWheelLocalizer extends HolonomicOdometry {
    public ThreeDeadWheelLocalizer(MotorEx leftDeadWheel, MotorEx rightDeadWheel, MotorEx backDeadWheel, Pose2d startingPose, double wheelRadius, double ticksPerRev, double wheelsDistance, double thirdWheelOffset) {
        super(
                () -> MathUtil.encoderTicksToMeter(leftDeadWheel.getCurrentPosition(), ticksPerRev, wheelRadius),
                () -> MathUtil.encoderTicksToMeter(rightDeadWheel.getCurrentPosition(), ticksPerRev, wheelRadius),
                () -> MathUtil.encoderTicksToMeter(backDeadWheel.getCurrentPosition(), ticksPerRev, wheelRadius),
                wheelsDistance,
                thirdWheelOffset
        );

        this.robotPose = startingPose;
    }

//    private double ticksToDistance(int ticks) {
//        return MathUtil.encoderTicksToMeter(ticks, this.ticksPerRev, this.wheelRadius);
//    }

//    private final double wheelRadius; // in meters
//    private final double ticksPerRev;
//    private final double wheelsDistance; // distance between left and right wheels
//    private final double thirdWheelOffset; // offset of the back wheel
//    private final MotorEx leftDeadWheel, rightDeadWheel, backDeadWheel;
//
//    private int lastLeftEncoderTicks, lastRightEncoderTicks, lastBackEncoderTicks;
//
//    private Pose2d pose; // Robot's global position
//    private ChassisSpeeds chassisSpeeds;
//    private long lastUpdateTime;
//
//    public ThreeDeadWheelLocalizer(MotorEx leftDeadWheel, MotorEx rightDeadWheel, MotorEx backDeadWheel, Pose2d startingPose, double wheelRadius, double ticksPerRev, double wheelsDistance, double thirdWheelOffset) {
//        this.leftDeadWheel = leftDeadWheel;
//        this.rightDeadWheel = rightDeadWheel;
//        this.backDeadWheel = backDeadWheel;
//
//        this.wheelRadius = wheelRadius;
//        this.ticksPerRev = ticksPerRev;
//        this.wheelsDistance = wheelsDistance;
//        this.thirdWheelOffset = thirdWheelOffset;
//
//        this.pose = startingPose;
//        this.chassisSpeeds = new ChassisSpeeds();
//        this.lastUpdateTime = System.currentTimeMillis();
//
//        this.resetPose();
//    }
//
//    public void update() {
//        long currentTime = System.currentTimeMillis();
//        double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // in seconds
//        lastUpdateTime = currentTime;
//
//        int leftEncoder = this.leftDeadWheel.getCurrentPosition();
//        int rightEncoder = this.rightDeadWheel.getCurrentPosition();
//        int backEncoder = this.backDeadWheel.getCurrentPosition();
//
//        // Compute deltas
//        int deltaLeft = leftEncoder - this.lastLeftEncoderTicks;
//        int deltaRight = rightEncoder - this.lastRightEncoderTicks;
//        int deltaBack = backEncoder - this.lastBackEncoderTicks;
//
//        // Update last encoder values
//        this.lastLeftEncoderTicks = leftEncoder;
//        this.lastRightEncoderTicks = rightEncoder;
//        this.lastBackEncoderTicks = backEncoder;
//
//        // Convert encoder ticks to distance
//        double leftDistance = ticksToDistance(deltaLeft);
//        double rightDistance = ticksToDistance(deltaRight);
//        double backDistance = ticksToDistance(deltaBack);
//
//        // Compute heading change
//        double deltaHeading = (rightDistance - leftDistance) / this.wheelsDistance;
//
//        // Compute robot-centric x and y movement
//        double deltaX = (leftDistance + rightDistance) / 2.0;
////        double deltaY = backDistance - deltaHeading * this.thirdWheelOffset;
//        double deltaY = backDistance - deltaHeading * this.thirdWheelOffset;
//
//        Vector2d deltaPose = new Vector2d(deltaX, deltaY).rotateBy(this.pose.getRotation().getDegrees());
//        HolonomicOdometry
//        this.pose = new Pose2d(
//                this.pose.getX() + deltaPose.getX(),
//                this.pose.getY() + deltaPose.getY(),
//                new Rotation2d(this.pose.getRotation().getRadians() + deltaHeading)
//        );
//
//        double leftVelocity = ticksToVelocity(deltaLeft, deltaTime);
//        double rightVelocity = ticksToVelocity(deltaRight, deltaTime);
//        double backVelocity = ticksToVelocity(deltaBack, deltaTime);
//
//        // Compute angular velocity (omega)
//        double omega = (rightVelocity - leftVelocity) / this.wheelsDistance;
//
//        // Compute linear velocities
//        double vX = (leftVelocity + rightVelocity) / 2.0;
//        double vY = backVelocity - omega * this.thirdWheelOffset;
//
//        this.chassisSpeeds.vxMetersPerSecond = vX;
//        this.chassisSpeeds.vyMetersPerSecond = vY;
//        this.chassisSpeeds.omegaRadiansPerSecond = omega;
//    }
//
//    private double ticksToVelocity(int deltaTicks, double deltaTime) {
//        double distance = ticksToDistance(deltaTicks); // in meters
//        return distance / deltaTime; // meters per second
//    }
//
//
//    public ChassisSpeeds getChassisSpeeds() {
//        return chassisSpeeds;
//    }
//
//    public Pose2d getPose() {
//        return this.pose;
//    }
//
//    public void setPose(Pose2d pose) {
//        this.pose = pose;
//    }
//
//    public void resetPose() {
//        this.pose = new Pose2d();
//        this.leftDeadWheel.resetEncoder();
//        this.rightDeadWheel.resetEncoder();
//        this.backDeadWheel.resetEncoder();
//    }
}

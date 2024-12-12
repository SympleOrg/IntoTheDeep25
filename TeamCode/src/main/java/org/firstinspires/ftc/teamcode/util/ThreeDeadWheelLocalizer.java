package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public class ThreeDeadWheelLocalizer {
    private final double wheelRadius; // in meters
    private final double ticksPerRev;
    private final double wheelsDistance; // distance between left and right wheels
    private final double thirdWheelOffset; // offset of the back wheel
    private final MotorEx leftDeadWheel, rightDeadWheel, backDeadWheel;

    private int lastLeftEncoderTicks, lastRightEncoderTicks, lastBackEncoderTicks;

    private Pose2d pose; // Robot's global position
    private ChassisSpeeds chassisSpeeds;
    private long lastUpdateTime;

    public ThreeDeadWheelLocalizer(MotorEx leftDeadWheel, MotorEx rightDeadWheel, MotorEx backDeadWheel, Pose2d startingPose, double wheelRadius, double ticksPerRev, double wheelsDistance, double thirdWheelOffset) {
        this.leftDeadWheel = leftDeadWheel;
        this.rightDeadWheel = rightDeadWheel;
        this.backDeadWheel = backDeadWheel;

        this.wheelRadius = wheelRadius;
        this.ticksPerRev = ticksPerRev;
        this.wheelsDistance = wheelsDistance;
        this.thirdWheelOffset = thirdWheelOffset;

        this.pose = startingPose;
        this.chassisSpeeds = new ChassisSpeeds();
        this.lastUpdateTime = System.currentTimeMillis();

        this.resetPose();
    }

    public void update() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // in seconds
        lastUpdateTime = currentTime;

        int leftEncoder = this.leftDeadWheel.getCurrentPosition();
        int rightEncoder = this.rightDeadWheel.getCurrentPosition();
        int backEncoder = this.backDeadWheel.getCurrentPosition();

        // Compute deltas
        int deltaLeft = leftEncoder - this.lastLeftEncoderTicks;
        int deltaRight = rightEncoder - this.lastRightEncoderTicks;
        int deltaBack = backEncoder - this.lastBackEncoderTicks;

        // Update last encoder values
        this.lastLeftEncoderTicks = leftEncoder;
        this.lastRightEncoderTicks = rightEncoder;
        this.lastBackEncoderTicks = backEncoder;

        // Convert encoder ticks to distance
        double leftDistance = ticksToDistance(deltaLeft);
        double rightDistance = ticksToDistance(deltaRight);
        double backDistance = ticksToDistance(deltaBack);

        // Compute heading change
        double deltaHeading = (rightDistance - leftDistance) / this.wheelsDistance;

        // Compute robot-centric x and y movement
        double deltaX = (leftDistance + rightDistance) / 2.0;
        double deltaY = backDistance - deltaHeading * this.thirdWheelOffset;

        // Convert to field-centric movement
        double cosHeading = Math.cos(this.pose.getHeading());
        double sinHeading = Math.sin(this.pose.getHeading());

        double fieldDeltaX = deltaX * cosHeading - deltaY * sinHeading;
        double fieldDeltaY = deltaX * sinHeading + deltaY * cosHeading;

        this.pose = this.pose.plus(new Transform2d(
                new Translation2d(fieldDeltaX, fieldDeltaY),
                new Rotation2d(deltaHeading)
        ));

        double leftVelocity = ticksToVelocity(deltaLeft, deltaTime);
        double rightVelocity = ticksToVelocity(deltaRight, deltaTime);
        double backVelocity = ticksToVelocity(deltaBack, deltaTime);

        // Compute angular velocity (omega)
        double omega = (rightVelocity - leftVelocity) / this.wheelsDistance;

        // Compute linear velocities
        double vX = (leftVelocity + rightVelocity) / 2.0;
        double vY = backVelocity - omega * this.thirdWheelOffset;

        this.chassisSpeeds.vxMetersPerSecond = vX;
        this.chassisSpeeds.vyMetersPerSecond = vY;
        this.chassisSpeeds.omegaRadiansPerSecond = omega;
    }

    private double ticksToVelocity(int deltaTicks, double deltaTime) {
        double distance = (deltaTicks * 2 * Math.PI * this.wheelRadius) / this.ticksPerRev; // in meters
        return distance / deltaTime; // meters per second
    }

    private double ticksToDistance(int ticks) {
        return (2 * Math.PI * this.wheelRadius * ticks) / this.ticksPerRev;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public Pose2d getPose() {
        return this.pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void resetPose() {
        this.pose = new Pose2d();
        this.leftDeadWheel.resetEncoder();
        this.rightDeadWheel.resetEncoder();
        this.backDeadWheel.resetEncoder();
    }
}

package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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

        leftDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);
        rightDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);
        backDeadWheel.encoder.setDirection(Motor.Direction.REVERSE);

        leftDeadWheel.resetEncoder();
        rightDeadWheel.resetEncoder();
        backDeadWheel.resetEncoder();

        updatePose(startingPose);
    }

    @Override
    public Pose2d getPose() {
        Pose2d pose2d = super.getPose();
        return new Pose2d(pose2d.getX(), -pose2d.getY(), Rotation2d.fromDegrees(360 - pose2d.getRotation().getDegrees()));
    }
}

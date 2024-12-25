package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DriveConstants {
    public static final double WHEEL_RADIUS = 0.045;
    public static final double WHEELS_DISTANCE = 0.19;

    public static final double ROBOT_RADIUS = 0.3; // in meter

    public static final double Ks = 0;
    public static final double MAX_VELOCITY = 0.75; // meter/sec
    public static final double MAX_ACCELERATION = 0.25; // meter/(sec^2)

    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(WHEELS_DISTANCE);
    public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);

    public static class DeedWheels {
        public static final double WHEEL_RADIUS = 0.024;
        public static final double TICKS_PER_REV = 2000;
        public static final double WHEELS_DISTANCE = 0.21; // TODO: find the values
        public static final double THIRD_WHEEL_OFFSET = 0.175; // TODO: find the values
    }

    public static class RamsetController {
        public static class Feedforward {
            public static final double Ks = 0.093;
            public static final double Kv = Math.pow(10, -6) + 0.01;
            public static final double Ka = Math.pow(10, -6) + 0.01;
        }

        @Config
        public static class PID {
            public static double Kp = 0.05;
            public static double Ki = 0;
            public static double Kd = 0;
        }
    }
}

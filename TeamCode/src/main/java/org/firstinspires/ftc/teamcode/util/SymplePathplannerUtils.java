package org.firstinspires.ftc.teamcode.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SymplePathplannerUtils {
    public static Pose2d toPose2d(com.arcrobotics.ftclib.geometry.Pose2d pose2d) {
        return new Pose2d(pose2d.getX(), pose2d.getY(), toRotation2d(pose2d.getRotation()));
    }

    public static com.arcrobotics.ftclib.geometry.Pose2d toPose2d(Pose2d pose2d) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getX(), pose2d.getY(), toRotation2d(pose2d.getRotation()));
    }

    public static Rotation2d toRotation2d(com.arcrobotics.ftclib.geometry.Rotation2d rotation2d) {
        return Rotation2d.fromDegrees(rotation2d.getDegrees());
    }

    public static com.arcrobotics.ftclib.geometry.Rotation2d toRotation2d(Rotation2d rotation2d) {
        return com.arcrobotics.ftclib.geometry.Rotation2d.fromDegrees(rotation2d.getDegrees());
    }
}

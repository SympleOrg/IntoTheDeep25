package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.RobotConstants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.AutoableDriveTrain;

public class Trajectories {
    public static AutoPath getPath(Paths paths, AutoableDriveTrain driveTrain, Pose2d startingPose) {
        switch (paths) {
            case PARK:
                return createParkTrajectory(driveTrain, startingPose);

            case CHAMBER:
                return createChamberTrajectory(driveTrain, startingPose);
        }

        return null;
    };

    public static AutoPath createParkTrajectory(AutoableDriveTrain driveTrain, Pose2d startingPose) {
        return new AutoPath.Builder(driveTrain, DriveConstants.TRAJECTORY_CONFIG)
                .setStartingPos(startingPose)
                .followPath(builder -> builder
                        .setNextPoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
                        .setNextPoint(new Pose2d(1, 0, Rotation2d.fromDegrees(0)))
                )
                .build();
    }

    public static AutoPath createChamberTrajectory(AutoableDriveTrain driveTrain, Pose2d startingPose) {
        return new AutoPath.Builder(driveTrain, DriveConstants.TRAJECTORY_CONFIG)
                .setStartingPos(startingPose)
                .followPath(builder -> builder
                        .setNextPoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
                        .setNextPoint(new Pose2d(-0.8, -0.35, Rotation2d.fromDegrees(0)))
                )
                .build();
    }

    public enum Paths {
        PARK,
        CHAMBER;
    }
}

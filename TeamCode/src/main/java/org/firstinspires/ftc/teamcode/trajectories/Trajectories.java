package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.AutoableDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;

import java.util.ArrayList;
import java.util.List;

public class Trajectories {
    public static AutoPath getPath(Paths paths, AutoableDriveTrain driveTrain) {
        switch (paths) {
            case PARK:
                return createParkTrajectory(driveTrain);
        }

        return null;
    };

    public static AutoPath createParkTrajectory(AutoableDriveTrain driveTrain) {
        return new AutoPath.Builder(driveTrain, DriveConstants.TRAJECTORY_CONFIG)
                .followPath(builder -> builder
                        .setNextPoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
                        .setNextPoint(new Pose2d(1, 0, Rotation2d.fromDegrees(0)))
                )
                .build();
    }

    public enum Paths {
        PARK;
    }
}

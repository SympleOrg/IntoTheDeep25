package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;

import java.util.ArrayList;
import java.util.List;

public class Trajectories {
    public static Trajectory testTrajectory() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(1, 0, Rotation2d.fromDegrees(0)));

        return TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.TRAJECTORY_CONFIG);
    }
}

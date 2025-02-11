package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.AutoRobotController;

import java.util.function.Function;

public enum Trajectories {
    CloseRed(CloseRedTrajectory::new, new Pose2d(0, -68.52, Math.toRadians(-90.00)));

    private final Function<AutoRobotController.SubsystemContainer, TrajectoryBase> trajectory;
    private final Pose2d startingPose;

    Trajectories(Function<AutoRobotController.SubsystemContainer, TrajectoryBase> trajectory, Pose2d startingPose) {
        this.trajectory = trajectory;
        this.startingPose = startingPose;
    }

    public void followTrajectory(AutoRobotController.SubsystemContainer subsystemContainer) {
        this.trajectory.apply(subsystemContainer).getTrajectory(startingPose).schedule();
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }
}

package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.AutoRobotController;

public abstract class TrajectoryBase {
    protected final AutoRobotController.SubsystemContainer subsystemContainer;

    public TrajectoryBase(AutoRobotController.SubsystemContainer subsystemContainer) {
        this.subsystemContainer = subsystemContainer;
    }

    public abstract Command getTrajectory(Pose2d startingPose);
}

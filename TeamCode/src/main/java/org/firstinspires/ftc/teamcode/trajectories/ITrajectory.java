package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.AutoRobotController;

public abstract class ITrajectory {
    protected final AutoRobotController.SubsystemContainer subsystemContainer;

    public ITrajectory(AutoRobotController.SubsystemContainer subsystemContainer) {
        this.subsystemContainer = subsystemContainer;
    }

    public abstract Command getPath();
}

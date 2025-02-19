package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.AutoRobotController;

import java.util.function.Function;
import java.util.function.Supplier;

public enum Trajectories {
    THREE_GAME_PIECE(ThreeGamePieceTrajectory::new);

    private Function<AutoRobotController.SubsystemContainer, ITrajectory> builder;

    Trajectories(Function<AutoRobotController.SubsystemContainer, ITrajectory> builder) {
        this.builder = builder;
    }

    public Command getTrajectory(AutoRobotController.SubsystemContainer subsystemContainer) {
        return builder.apply(subsystemContainer).getPath();
    }
}

package org.firstinspires.ftc.teamcode.trajectories;

import android.util.Pair;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.AutoableDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class AutoPath {
    private final List<Command> actions;
    private AutoPath(List<Command> actions) {
        this.actions = actions;
    }

    public void follow() {
        SequentialCommandGroup sequentialCommandGroup = new SequentialCommandGroup();

        for(Command action : actions) {
            sequentialCommandGroup.addCommands(action);
        }

        sequentialCommandGroup.schedule();
    }

    public void drawPath(Canvas canvas) {
        Pose2d lastPos = null;
        for (Command command : actions) {
            if(command instanceof FollowTrajectoryCommand) {
                lastPos = null;
                FollowTrajectoryCommand followTrajectoryCommand = (FollowTrajectoryCommand) command;
                Trajectory trajectory = followTrajectoryCommand.getTrajectory();
                List<Pose2d> points = trajectory.getStates().stream().map(state ->
                        new Pose2d(
                                MathUtil.meterToInch(state.poseMeters.getX()),
                                MathUtil.meterToInch(state.poseMeters.getY()),
                                state.poseMeters.getRotation()
                        )
                ).collect(Collectors.toList());

                for (Pose2d p : points) {
                    canvas.setFill("#0000ff");
                    canvas.fillCircle(p.getX(), p.getY(), 1);
                    if(lastPos != null) {
                        canvas.setStrokeWidth(1);
                        canvas.setStroke("#ff0000");
                        canvas.strokeLine(lastPos.getX(), lastPos.getY(), p.getX(), p.getY());
                    }
                    lastPos = p;
                }
            } else if(lastPos != null) {
                canvas.setFill("#0000ff");
                canvas.fillCircle(lastPos.getX(), lastPos.getY(), 1);
            }
        }
    }

    public static class Builder {
        private final List<Command> actions;
        private final AutoableDriveTrain driveTrain;
        private final TrajectoryConfig trajectoryConfig;

        private Pose2d startingPos = null;

        public Builder(AutoableDriveTrain driveTrain, TrajectoryConfig trajectoryConfig) {
            this.actions = new ArrayList<>();
            this.driveTrain = driveTrain;
            this.trajectoryConfig = trajectoryConfig;
        }

        public Builder setStartingPos(Pose2d pose2d) {
            this.startingPos = pose2d;
            return this;
        }

        public Builder doAction(Command action) {
            this.actions.add(action);
            return this;
        };

        public Builder doAction(Runnable runnable) {
            this.actions.add(new InstantCommand(runnable));
            return this;
        };

        public Builder followPath(Trajectory trajectory) {
            this.actions.add(new FollowTrajectoryCommand(this.driveTrain, trajectory));
            return this;
        }

        public Builder followPath(Consumer<TrajectoryBuilder> builder) {
            TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();
            builder.accept(trajectoryBuilder);
            return this.followPath(trajectoryBuilder.build(this.trajectoryConfig));
        }

        public AutoPath build() {
            return new AutoPath(this.actions);
        }
    }

    public static class TrajectoryBuilder {
        private final List<Pose2d> points;

        private TrajectoryBuilder() {
            this.points = new ArrayList<>();
        }

        public TrajectoryBuilder setNextPoint(Pose2d pose2d) {
            this.points.add(pose2d);
            return this;
        }

        public Trajectory build(TrajectoryConfig config) {
            return TrajectoryGenerator.generateTrajectory(this.points, config);
        }
    }
}

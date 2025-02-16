package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.Collections;
import java.util.Set;

public class CloseRedTrajectory extends TrajectoryBase {
    public CloseRedTrajectory(AutoRobotController.SubsystemContainer subsystemContainer) {
        super(subsystemContainer);
    }

    @Override
    public Command getTrajectory(Pose2d startingPose) {
        TrajectoryActionBuilder driveToScorePreloadSample = this.subsystemContainer.getMecanumDriveSubsystem().actionBuilder(startingPose)
                .lineToYConstantHeading(MathUtil.metersToInch(-0.65))
//                .strafeTo(new Vector2d(Math.cos(Math.PI / 4) * MathUtil.metersToInch(1.05), Math.sin(-Math.PI / 4) * MathUtil.metersToInch(1.05)))
                .setTangent(Math.PI / 4)
                .splineToConstantHeading(new Vector2d(MathUtil.metersToInch(1.15), MathUtil.metersToInch(0)), Math.PI * 0.75)
                .lineToYConstantHeading(MathUtil.metersToInch(-0.65));

//                .lineToYConstantHeading(MathUtil.metersToInch(startingPose.position.y + 0.1));
//        TrajectoryActionBuilder driveToPickUpSecondSample = driveToScorePreloadSample.endTrajectory().fresh().splineToLinearHeading(new Pose2d(-48.74, -37.62, Math.toRadians(90.00)), Math.toRadians(78.16));
//        TrajectoryActionBuilder driveToPickUpSecondSample1 = driveToPickUpSecondSample.endTrajectory().fresh().setTangent(Math.toRadians(240));
//        TrajectoryActionBuilder DriveToScoreSecondsample = driveToPickUpSecondSample1.endTrajectory().fresh()   .splineToSplineHeading(new Pose2d(-54.00, -55.21, Math.toRadians(50.00)), Math.toRadians(78.60));
//        TrajectoryActionBuilder drivetoPickUpthirdsample = DriveToScoreSecondsample.endTrajectory().fresh();
        return new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSample.endTrajectory().build(), Collections.emptySet())
        );
    }
}

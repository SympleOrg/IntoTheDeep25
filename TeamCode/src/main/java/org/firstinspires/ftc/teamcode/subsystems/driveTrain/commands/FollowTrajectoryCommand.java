package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.AutoableDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;

public class FollowTrajectoryCommand extends RamseteCommand {
    private final Trajectory trajectory;

    public FollowTrajectoryCommand(AutoableDriveTrain subsystem, Trajectory trajectory) {
        super(
                trajectory,
//                subsystem::getPosition,
                () -> new Pose2d(subsystem.getPosition().getTranslation(), subsystem.getPosition().getRotation().rotateBy(Rotation2d.fromDegrees(180))),
                new RamseteController(DriveConstants.RamsetController.RamsetTurning.Kb, DriveConstants.RamsetController.RamsetTurning.Kz),
                new SimpleMotorFeedforward(
                        DriveConstants.RamsetController.Feedforward.Ks,
                        DriveConstants.RamsetController.Feedforward.Kv,
                        DriveConstants.RamsetController.Feedforward.Ka
                ),
                DriveConstants.DRIVE_KINEMATICS,
                subsystem::getWheelSpeeds,
                new PIDController(
                        DriveConstants.RamsetController.PID.Kp,
                        DriveConstants.RamsetController.PID.Ki,
                        DriveConstants.RamsetController.PID.Kd
                ),
                new PIDController(
                        DriveConstants.RamsetController.PID.Kp,
                        DriveConstants.RamsetController.PID.Ki,
                        DriveConstants.RamsetController.PID.Kd
                ),
                subsystem::moveSideMotors
//                (left, right) -> subsystem.moveSideMotors(right, left)
        );

        addRequirements(subsystem);

        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}

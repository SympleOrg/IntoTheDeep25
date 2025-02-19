package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.StrafeInAngleMecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.TurnToFLLCommand;

public abstract class ITrajectory {
    protected final AutoRobotController.SubsystemContainer subsystemContainer;

    public ITrajectory(AutoRobotController.SubsystemContainer subsystemContainer) {
        this.subsystemContainer = subsystemContainer;
    }

    protected Command drive(double cm) {
        return new DriveDistanceDriveCommand(this.subsystemContainer.getMecanumDriveSubsystem(), cm / -100);
    }

    protected Command rotate(double deg) {
        return new TurnToFLLCommand(this.subsystemContainer.getMecanumDriveSubsystem(), Rotation2d.fromDegrees(deg))
                .andThen(new InstantCommand(() -> StrafeInAngleMecanumCommand.updateAngle(deg)));
    }

    protected Command strafe(double deg, double cm) {
        return new StrafeInAngleMecanumCommand(this.subsystemContainer.getMecanumDriveSubsystem(), deg, cm / 100);
    }

    protected Command driveBackwards(double cm) {
        return new StrafeInAngleMecanumCommand(this.subsystemContainer.getMecanumDriveSubsystem(), 0, cm / 100);
    }

    public abstract Command getPath();
}

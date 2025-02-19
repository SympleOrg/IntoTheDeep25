package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.StrafeInAngleMecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.TurnToFLLCommand;

public class ThreeGamePieceTrajectory extends ITrajectory {
    public ThreeGamePieceTrajectory(AutoRobotController.SubsystemContainer subsystemContainer) {
        super(subsystemContainer);
    }

    @Override
    public Command getPath() {
        return new SequentialCommandGroup(
                drive(-0.7),
                // ELEVATOR SHIT
                drive(0.2),
                strafe(-90, 0.7),
                drive(-0.8),
                strafe(90, 0.1),
                rotate(90), // Rotate robot to push game pieces
                rotate(90), // Rotate robot to push game pieces
                strafe(-90, 0.18),
                 new WaitCommand(500),
                drive(-1.2),
                drive(1.4),
                strafe(90, 0.4),
                rotate(10),
                drive(-1.2)
        );
    }

    private Command drive(double meters) {
        return new DriveDistanceDriveCommand(this.subsystemContainer.getMecanumDriveSubsystem(), meters);
    }

    private Command rotate(double deg) {
        return new TurnToFLLCommand(this.subsystemContainer.getMecanumDriveSubsystem(), Rotation2d.fromDegrees(deg));
    }

    private Command strafe(double deg, double meters) {
        return new StrafeInAngleMecanumCommand(this.subsystemContainer.getMecanumDriveSubsystem(), deg, meters);
    }
}

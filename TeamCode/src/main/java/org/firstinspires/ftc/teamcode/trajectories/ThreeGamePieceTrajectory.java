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
//        return new SequentialCommandGroup(
//                drive(-0.7),
//                // ELEVATOR SHIT
//                drive(0.2),
//                strafe(-90, 0.7),
//                drive(-0.8),
//                strafe(90, 0.1),
//                rotate(180), // Rotate robot to push game pieces
//                strafe(90, 0.1),
//                 new WaitCommand(500),
//                drive(-1.2),
//                drive(1.1),
//                strafe(90, 0.175),
////                rotate(10),
//                drive(-0.4),
//                new WaitCommand(2000),
//                drive(0.45)
//                // take game piece
//        );
        return new SequentialCommandGroup(
                strafe(180, 49.02739212103907),
                strafe(0, 7.280109889280519),
                strafe(90, 60.02221947941518),
                strafe(180, 44.03124237432849),
                rotate(180),
                strafe(90, 29.068883707497264),
                strafe(0, 100.01639234135716),
                strafe(-180, 122.3478647136925),
                strafe(90, 17.11724276862369),
                strafe(0, 100.01562404644207)
        );
    }

//
//    @Override
//    public Command getPath() {
//        return new SequentialCommandGroup();
//    }
}

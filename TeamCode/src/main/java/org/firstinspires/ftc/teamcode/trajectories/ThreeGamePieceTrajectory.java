package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.RobotConstants;
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
                this.subsystemContainer.getClawSubsystem().moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                strafe(180, 35.5),
                this.subsystemContainer.getElevatorSubsystem().goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP, subsystemContainer.getIntakePitchJointSubsystem()),
                strafe(180, 39.5), // quick FIX
                 // fwd to submersable
                scoreOnChamber(),
               strafe(81.55, 90.3),
         strafe(175, 64),
               rotate(180),
                strafe(90, 28).withTimeout(2000),
             strafe(0, 93).withTimeout(2000), // Return element
////                // Second one
////                strafe(160, 103).withTimeout(2000),
////                strafe(90, 25).withTimeout(2000), // Strafe to it
////                strafe(0, 100),

                    strafe(160, 25), // Go back and center
                    strafe(0, 55), // go to collect // TODO CHANGE THIS
                    waitForSpecimen(),
                    strafe(0, 10).withTimeout(1500),
                    takeFromHuman(),
                    strafe(-110, 125),
                    rotate(180),
                    strafe(0, 50).withTimeout(1000),
                    scoreOnChamber()
                // Third One
////                strafe(160, 90),
////                strafe(90, 25).withTimeout(2000), // Strafe to it
////                strafe(0, 93)
////                strafe(200, 40),
////                strafe(0, 75),
////                scoreOnChamber()
        );
    }


//
//    @Override
//    public Command getPath() {
//        return new SequentialCommandGroup();
//    }
}

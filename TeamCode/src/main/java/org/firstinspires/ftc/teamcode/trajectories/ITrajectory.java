package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.RobotConstants;
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

    protected Command waitForSpecimen() {
        return new WaitUntilCommand(this.subsystemContainer.getClawSubsystem()::isSampleDetected)
                .raceWith(new WaitCommand(1500));
    }

    protected Command takeFromHuman() {
        return new SequentialCommandGroup(
                this.subsystemContainer.getClawSubsystem().moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                new WaitCommand(250),
                this.subsystemContainer.getElevatorSubsystem().goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP, subsystemContainer.getIntakePitchJointSubsystem())
        );
    }

    protected Command scoreOnChamber() {
        return new SequentialCommandGroup(
                this.subsystemContainer.getElevatorSubsystem().scoreOnChamber(),
                this.subsystemContainer.getClawSubsystem().moveToState(RobotConstants.ClawConstants.ClawState.OPEN),
                new WaitCommand(250),
                this.subsystemContainer.getElevatorSubsystem().goToState(RobotConstants.ElevatorConstants.ElevatorState.REST, this.subsystemContainer.getIntakePitchJointSubsystem())
        );
    }

    public abstract Command getPath();
}

package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;

import java.util.function.Supplier;

public class Dis1Autos {
    private final MecanumDriveSubsystem driveBase;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;

    public Dis1Autos(MecanumDriveSubsystem mecanumDriveSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, ScorerSubsystem scorerSubsystem) {
        this.driveBase = mecanumDriveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.scorerSubsystem = scorerSubsystem;
    }

    public Command chamberAuto() {
        return new SequentialCommandGroup(
                driveDist(-0.58),
                this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP),
                driveDist(-0.1).withTimeout(3000),
                scoreChamber(),
                driveDist(0.48),
                rotate(65),
                driveDist(0.80),
                rotate(25),
                driveDist(-1.25).withTimeout(2000),
                driveDist(0.55),
                new WaitCommand(2000),
                driveDist(-0.85),
                new SequentialCommandGroup(
                        new WaitCommand(1000),
                        this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                        new WaitCommand(2000),
                        this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP)
                ),
                driveDist(0.4),
                rotate(-90),
                driveDist(-0.7),
                rotate(-90),
                driveDist(-0.33).withTimeout(3000),
                scoreChamber()
        );
    }

    public Command basketAuto() {
        return new SequentialCommandGroup(
                this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.BASKET_TOP),
                 driveDist(-0.46),
                new WaitCommand(500),
                this.scorerSubsystem.moveToState(RobotConstants.ScorerConstants.ScorerState.SCORE),
                new WaitCommand(300),
                this.scorerSubsystem.moveToState(RobotConstants.ScorerConstants.ScorerState.TAKE),
                new WaitCommand(100),
                this.scorerSubsystem.moveToState(RobotConstants.ScorerConstants.ScorerState.SCORE),
                new WaitCommand(1000),
                driveDist(0.65),
                new WaitCommand(300),
                this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.REST),
                rotate(-90),
                driveDist(0.58)
        );
    }

    private Command driveDist(double meters) {
        return new DriveDistanceDriveCommand(this.driveBase, meters);
    }

    private Command rotate(double deg) {
        return new RotateRobotByDegCommand(this.driveBase, deg);
    }

    private Command scoreChamber() {
        return new SequentialCommandGroup(
                new WaitCommand(350),
                this.elevatorSubsystem.scoreOnChamber(),
                this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.OPEN),
                new WaitCommand(400),
                new ParallelCommandGroup(
                        this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.REST),
                        rotate(90)
                )
        );
    }
}

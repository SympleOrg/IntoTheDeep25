package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotConstants.*;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeXJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeYJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;

public class DriverCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;
    private final IntakeXJointSubsystem intakeXJointSubsystem;
    private final IntakeYJointSubsystem intakeYJointSubsystem;

    public DriverCommands(
            MecanumDriveSubsystem mecanumDriveSubsystem,
            ClawSubsystem clawSubsystem,
            ScorerSubsystem scorerSubsystem,
            IntakeSubsystem intakeSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ExtenderSubsystem extenderSubsystem,
            IntakeXJointSubsystem intakeXJointSubsystem,
            IntakeYJointSubsystem intakeYJointSubsystem
    ) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.scorerSubsystem = scorerSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.extenderSubsystem = extenderSubsystem;
        this.intakeXJointSubsystem = intakeXJointSubsystem;
        this.intakeYJointSubsystem = intakeYJointSubsystem;
    }

    public Command intakeToScorer() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        this.elevatorSubsystem.goToState(ElevatorConstants.ElevatorState.REST),
                        this.scorerSubsystem.moveToState(ScorerConstants.ScorerState.TAKE),
                        this.intakeXJointSubsystem.moveToState(IntakeJointConstants.JointXState.CLOSED),
                        this.extenderSubsystem.goToRest()
                ).withTimeout(1000),
                new SequentialCommandGroup(
                        // TODO: fix this
//                        this.intakeSubsystem.goToState(IntakeConstants.IntakeState.DROP)
//                                .withTimeout(1000),
                        this.intakeXJointSubsystem.moveToState(IntakeJointConstants.JointXState.HUMAN_PLAYER)
                )
        );
    }

    public Command scoreBasket() {
        return this.scorerSubsystem.toggleState();
    }

    public Command toggleClaw() {
        return this.clawSubsystem.toggleState();
    }

    public Command goToDefaultStates() {
        return new ParallelCommandGroup(
            this.scorerSubsystem.moveToState(ScorerConstants.ScorerState.TAKE),
            this.elevatorSubsystem.goToDefaultState(),
            this.clawSubsystem.moveToState(ClawConstants.ClawState.OPEN),
            this.extenderSubsystem.goToRest()
        ).withTimeout(3000);
    }
}

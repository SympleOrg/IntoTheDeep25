package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.*;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakePitchJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeRollJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;

import java.util.HashMap;

public class DriverCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;
    private final IntakePitchJointSubsystem intakePitchJointSubsystem;
    private final IntakeRollJointSubsystem intakeRollJointSubsystem;

    public DriverCommands(
            MecanumDriveSubsystem mecanumDriveSubsystem,
            ClawSubsystem clawSubsystem,
            ScorerSubsystem scorerSubsystem,
            IntakeSubsystem intakeSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ExtenderSubsystem extenderSubsystem,
            IntakePitchJointSubsystem intakePitchJointSubsystem,
            IntakeRollJointSubsystem intakeRollJointSubsystem
    ) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.scorerSubsystem = scorerSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.extenderSubsystem = extenderSubsystem;
        this.intakePitchJointSubsystem = intakePitchJointSubsystem;
        this.intakeRollJointSubsystem = intakeRollJointSubsystem;
    }

    public Command toggleClaw() {
        return this.clawSubsystem.toggleState();
    }

    public Command goToDefaultActionStates() {
        return new ParallelCommandGroup(
            this.scorerSubsystem.moveToState(ScorerConstants.ScorerState.TAKE),
            this.elevatorSubsystem.goToDefaultState(),
            this.clawSubsystem.moveToState(ClawConstants.ClawState.OPEN),
            this.extenderSubsystem.goToRest()
        ).withTimeout(3000);
    }

    public Command resetDrive() {
        return this.mecanumDriveSubsystem.resetRotation();
    }

    public Command elevatorGoMax() {
        return this.elevatorSubsystem.goToState(ElevatorConstants.ElevatorState.BASKET_TOP);
    }

    public Command elevatorGoMin() {
        return this.elevatorSubsystem.goToState(ElevatorConstants.ElevatorState.REST);
    }

    public Command toggleChamberElevator() {
        return new SelectCommand(
                new HashMap<Object, Command>(){{
                    put("up", DriverCommands.this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP));
                    put("down", DriverCommands.this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.REST));
                    put("score", new ParallelCommandGroup(
                            DriverCommands.this.elevatorSubsystem.scoreOnChamber(),
                            new SequentialCommandGroup(
                                    new WaitCommand(1000),
                                    DriverCommands.this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.OPEN)
                            )
                    ));
                    put("no", new InstantCommand(() -> {}));
                }},
                () -> {
                    if(this.clawSubsystem.getState() == RobotConstants.ClawConstants.ClawState.CLOSE
                            && (this.elevatorSubsystem.getState() == RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP || this.elevatorSubsystem.getState() == RobotConstants.ElevatorConstants.ElevatorState.SCORE_BOTTOM)) {
                        return "score";
                    } else if(this.elevatorSubsystem.getState() == RobotConstants.ElevatorConstants.ElevatorState.HUMAN_PLAYER || this.elevatorSubsystem.getState() == RobotConstants.ElevatorConstants.ElevatorState.REST) {
                        return "up";
                    } else {
                        return "down";
                    }
                }
        );
    }
}

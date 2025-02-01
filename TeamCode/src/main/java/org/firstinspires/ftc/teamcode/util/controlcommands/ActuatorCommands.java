package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeXJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeYJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;

import java.util.HashMap;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;
    private final IntakeXJointSubsystem intakeXJointSubsystem;
    private final IntakeYJointSubsystem intakeYJointSubsystem;

    public ActuatorCommands(
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

    public Command takeGamePiece() {
//        return new ParallelCommandGroup(
//                this.intakeSubsystem.goToState(RobotConstants.IntakeConstants.IntakeState.TAKE),
//                new ConditionalCommand(
//                        this.intakeXJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointXState.TAKE),
//                        new InstantCommand(),
//                        () -> this.intakeXJointSubsystem.getState() == RobotConstants.IntakeJointConstants.JointXState.PRETAKE
//                )
//        );
        // TODO: change this placeholder
        return new InstantCommand();
    }

    public Command dropGamePiece() {
//        return this.intakeSubsystem.goToState(RobotConstants.IntakeConstants.IntakeState.DROP);
        // TODO: change this placeholder
        return new InstantCommand();
    }

    public Command stopIntake() {
//        return new ParallelCommandGroup(
//                this.intakeSubsystem.goToState(RobotConstants.IntakeConstants.IntakeState.IDLE),
//                new ConditionalCommand(
//                        this.intakeXJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointXState.PRETAKE),
//                        new InstantCommand(),
//                        () -> this.intakeXJointSubsystem.getState() == RobotConstants.IntakeJointConstants.JointXState.TAKE
//                )
//        );
        // TODO: change this placeholder
        return new InstantCommand();
    }

    public Command toggleChamberElevator() {
        return new SelectCommand(
                new HashMap<Object, Command>(){{
                    put("up", ActuatorCommands.this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP));
                    put("down", ActuatorCommands.this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.REST));
                    put("score", new ParallelCommandGroup(
                            ActuatorCommands.this.elevatorSubsystem.scoreOnChamber(),
                            new SequentialCommandGroup(
                                    new WaitCommand(1000),
                                    ActuatorCommands.this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.OPEN)
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

    public Command goToTopBasket() {
        return this.elevatorSubsystem.toggleStates(
                RobotConstants.ElevatorConstants.ElevatorState.BASKET_TOP,
                RobotConstants.ElevatorConstants.ElevatorState.REST
        );
    }

    public Command goToBottomBasket() {
        return this.elevatorSubsystem.toggleStates(
                RobotConstants.ElevatorConstants.ElevatorState.BASKET_BOTTOM,
                RobotConstants.ElevatorConstants.ElevatorState.REST
        );
    }

    public Command moveJointToHumanPlayer() {
        return this.intakeXJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointXState.HUMAN_PLAYER);
    }

    public Command togglePretake() {
        return this.intakeXJointSubsystem.toggleStates(
                RobotConstants.IntakeJointConstants.JointXState.PRETAKE,
                RobotConstants.IntakeJointConstants.JointXState.CLOSED
        );
    }

    public Command controlExtenderWithJoystick(GamepadEx gamepadEx) {
        return this.extenderSubsystem.moveWithJoyStick(gamepadEx);
    }
}

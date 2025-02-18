package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakePitchJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeRollJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;
    private final IntakePitchJointSubsystem intakePitchJointSubsystem;
    private final IntakeRollJointSubsystem intakeRollJointSubsystem;

    public ActuatorCommands(
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

    public Command toggleIntake() {
        return this.intakeSubsystem.toggleStates(RobotConstants.IntakeConstants.IntakeState.OPEN, RobotConstants.IntakeConstants.IntakeState.CLOSE);
    }

    public Command rotateCW() {
        return this.intakeRollJointSubsystem.rotateCW();
    }

    public Command rotateC() {
        return this.intakeRollJointSubsystem.rotateC();
    }

    public Command intakeGoTake() {
        return this.intakePitchJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointPitchState.TAKE);
    }

    public Command intakeGoHuman() {
        return this.intakePitchJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointPitchState.HUMAN_PLAYER);
    }

    public Command intakeGoBasket() {
        return this.intakePitchJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointPitchState.BASKET);
    }

    public Command toggleBasket() {
        return this.scorerSubsystem.toggleState();
    }

    public Command controlExtenderWithJoystick(GamepadEx gamepadEx) {
        return this.extenderSubsystem.moveWithJoyStick(gamepadEx);
    }
}

package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;
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

    public Command openIntake() {
        return this.intakeSubsystem.goToState(RobotConstants.IntakeConstants.IntakeState.OPEN);
    }

    public Command closeIntake() {
        return this.intakeSubsystem.goToState(RobotConstants.IntakeConstants.IntakeState.CLOSE);
    }

    public Command rotateCW() {
        return this.intakeYJointSubsystem.rotateCW();
    }

    public Command rotateC() {
        return this.intakeYJointSubsystem.rotateC();
    }

    public Command intakeGoTake() {
        return this.intakeXJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointXState.TAKE);
    }

    public Command intakeGoHuman() {
        return this.intakeXJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointXState.HUMAN_PLAYER);
    }

    public Command intakeGoBasket() {
        return this.intakeXJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointXState.BASKET);
    }

    public Command toggleBasket() {
        return this.scorerSubsystem.toggleState();
    }

    public Command controlExtenderWithJoystick(GamepadEx gamepadEx) {
        return this.extenderSubsystem.moveWithJoyStick(gamepadEx);
    }
}

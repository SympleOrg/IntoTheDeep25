package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.MecanumArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

import java.util.HashMap;

public class TeleOpRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;
    private final IntakeJointSubsystem intakeJointSubsystem;

    private TeleOpRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor, String logFilePrefix, boolean logData) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        if(teamColor == null) {
            RuntimeException exception = new RuntimeException("Team color cannot be null!");
            this.getDataLogger().addThrowable(exception);
            throw exception;
        }

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), new Pose2d(), this.getTelemetry(), this.getDataLogger());
        this.clawSubsystem = new ClawSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.scorerSubsystem = new ScorerSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeSubsystem = new IntakeSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.elevatorSubsystem = new ElevatorSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.extenderSubsystem = new ExtenderSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeJointSubsystem = new IntakeJointSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
    }

    @Override
    public void createKeyBindings() {
        this.actionController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SelectCommand(
                                new HashMap<Object, Command>(){{
                                    put("up", TeleOpRobotController.this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP));
                                    put("down", TeleOpRobotController.this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorState.REST));
                                    put("score", new ParallelCommandGroup(
                                            TeleOpRobotController.this.elevatorSubsystem.scoreOnChamber(),
                                            new SequentialCommandGroup(
                                                    new WaitCommand(1000),
                                                    TeleOpRobotController.this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.OPEN)
                                            )
                                    ));
                                    put("no", new InstantCommand(() -> {}));
                                }},
                                () -> {
                                    if(this.clawSubsystem.getState() == RobotConstants.ClawConstants.ClawState.CLOSE
                                       && (this.elevatorSubsystem.getState() == RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP || this.elevatorSubsystem.getState() == RobotConstants.ElevatorConstants.ElevatorState.SCORE_BOTTOM)) {
                                        return "score";
                                    } else if (this.clawSubsystem.getState() == RobotConstants.ClawConstants.ClawState.OPEN) {
                                        return "down";
                                    } else {
                                        return "up";
                                    }
                                }
                        )
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(
                        this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                        this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.OPEN)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(
                        this.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.PRETAKE),
                        this.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.CLOSED)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        this.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.HUMAN_PLAYER)
                );

        new Trigger(() -> this.actionController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whenActive(
                        new ParallelCommandGroup(
                                this.intakeSubsystem.setState(RobotConstants.IntakeConstants.IntakeState.TAKE),
                                new ConditionalCommand(
                                        this.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.TAKE),
                                        new InstantCommand(),
                                        () -> this.intakeJointSubsystem.getState() == RobotConstants.IntakeJointConstants.JointState.PRETAKE
                                )
                        )
                );

        new Trigger(() -> this.actionController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whenActive(
                        this.intakeSubsystem.setState(RobotConstants.IntakeConstants.IntakeState.DROP)
                );

        new Trigger(() -> this.actionController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.1 && this.actionController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.1)
                .whileActiveContinuous(
                    new ParallelCommandGroup(
                            this.intakeSubsystem.setState(RobotConstants.IntakeConstants.IntakeState.IDLE),
                            new ConditionalCommand(
                                    this.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.PRETAKE),
                                    new InstantCommand(),
                                    () -> this.intakeJointSubsystem.getState() == RobotConstants.IntakeJointConstants.JointState.TAKE
                            )
                    )
            );
    }

    @Override
    public void initialize() {
        this.mecanumDriveSubsystem.setDefaultCommand(new MecanumArcadeDriveCommand(this.mecanumDriveSubsystem, this.driverController));
        this.extenderSubsystem.setDefaultCommand(this.extenderSubsystem.moveWithJoyStick(this.actionController));
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        this.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.CLOSED).schedule();
    }

    @Override
    public void run() {

    }

    @Override
    public void postRun() {

    }

    public static class Builder extends RobotControllerBase.Builder {
        private TeamColor teamColor;

        public Builder() {
            this.logFilePrefix = "TeleOp";
        }

        public Builder teamColor(TeamColor value) {
            this.teamColor = value;
            return this;
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        @Override
        public TeleOpRobotController build() {
            return new TeleOpRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.teamColor, this.logFilePrefix, this.logData);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.MecanumArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakePitchJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeRollJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;
import org.firstinspires.ftc.teamcode.util.controlcommands.ActuatorCommands;
import org.firstinspires.ftc.teamcode.util.controlcommands.DriverCommands;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;
import org.firstinspires.ftc.teamcode.RobotConstants.*;

public class TeleOpRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;
    private final IntakePitchJointSubsystem intakePitchJointSubsystem;
    private final IntakeRollJointSubsystem intakeRollJointSubsystem;

    private final DriverCommands driverCommands;
    private final ActuatorCommands actuatorCommands;

    private TeleOpRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor, String logFilePrefix, boolean logData) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        if(teamColor == null) {
            RuntimeException exception = new RuntimeException("Team color cannot be null!");
            this.getDataLogger().addThrowable(exception);
            throw exception;
        }

        RobotPositionManager.getInstance().setRotation(AutoRobotController.endRotation.plus(Rotation2d.fromDegrees(180)));

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.clawSubsystem = new ClawSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.scorerSubsystem = new ScorerSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeSubsystem = new IntakeSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.elevatorSubsystem = new ElevatorSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.extenderSubsystem = new ExtenderSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakePitchJointSubsystem = new IntakePitchJointSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeRollJointSubsystem = new IntakeRollJointSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());

        this.driverCommands = new DriverCommands(
                mecanumDriveSubsystem,
                clawSubsystem,
                scorerSubsystem,
                intakeSubsystem,
                elevatorSubsystem,
                extenderSubsystem,
                intakePitchJointSubsystem,
                intakeRollJointSubsystem
        );

        this.actuatorCommands = new ActuatorCommands(
                mecanumDriveSubsystem,
                clawSubsystem,
                scorerSubsystem,
                intakeSubsystem,
                elevatorSubsystem,
                extenderSubsystem,
                intakePitchJointSubsystem,
                intakeRollJointSubsystem
        );
    }

    @Override
    public void createKeyBindings() {
        this.driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> this.mecanumDriveSubsystem.setDriveSpeed(DriveConstants.DriveSpeed.NORMAL));

        this.driverController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> this.mecanumDriveSubsystem.setDriveSpeed(DriveConstants.DriveSpeed.SLOW));

        this.driverController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(this.driverCommands.toggleChamberElevator());

        this.driverController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(this.driverCommands.toggleClaw());

        this.driverController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.driverCommands.resetDrive());

        new Trigger()
                .or(this.driverController.getGamepadButton(GamepadKeys.Button.DPAD_UP))
                .or(this.driverController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN))
                .or(this.driverController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT))
                .or(this.driverController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(this.driverCommands.goToDefaultActionStates());

        new Trigger(() -> this.driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.1)
                .whenActive(this.driverCommands.elevatorGoMax());

        new Trigger(() -> this.driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.1)
                .whenActive(this.driverCommands.elevatorGoMin());

        this.actionController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this.actuatorCommands.rotateCW());

        this.actionController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(this.actuatorCommands.rotateC());

        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(this.actuatorCommands.intakeGoBasket());

        this.actionController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.actuatorCommands.intakeGoHuman());

        this.actionController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.actuatorCommands.intakeGoTake());

        this.actionController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(this.actuatorCommands.toggleBasket());

        new Trigger(() -> this.actionController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whenActive(this.actuatorCommands.toggleIntake());

        new Trigger(() -> Math.abs(this.actionController.getRightY()) > 0.1)
                .whenActive(this.actuatorCommands.controlExtenderWithJoystick(this.actionController));
    }

    @Override
    public void initialize() {
        this.mecanumDriveSubsystem.setDefaultCommand(new MecanumArcadeDriveCommand(this.mecanumDriveSubsystem, this.driverController));
        this.extenderSubsystem.setDefaultCommand(this.actuatorCommands.controlExtenderWithJoystick(this.actionController));
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        this.intakeSubsystem.goToState(IntakeConstants.IntakeState.CLOSE).schedule();
        this.intakeRollJointSubsystem.moveToState(IntakeJointConstants.JointRollState.ZERO).schedule();
        this.intakePitchJointSubsystem.moveToState(IntakeJointConstants.JointPitchState.BASKET).schedule();
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

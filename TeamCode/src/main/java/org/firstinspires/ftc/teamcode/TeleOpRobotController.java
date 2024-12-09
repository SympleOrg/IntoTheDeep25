package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.MecanumArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class TeleOpRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ScorerSubsystem scorerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;

    private TeleOpRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor, String logFilePrefix, boolean logData) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        if(teamColor == null) {
            RuntimeException exception = new RuntimeException("Team color cannot be null!");
            this.getDataLogger().addThrowable(exception);
            throw exception;
        }

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.clawSubsystem = new ClawSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.scorerSubsystem = new ScorerSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeSubsystem = new IntakeSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.elevatorSubsystem = new ElevatorSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.extenderSubsystem = new ExtenderSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
    }

    @Override
    public void createKeyBindings() {
        this.actionController.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                        this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.OPEN)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(
                        this.scorerSubsystem.moveToState(RobotConstants.ScorerConstants.ScorerState.TAKE),
                        this.scorerSubsystem.moveToState(RobotConstants.ScorerConstants.ScorerState.SCORE)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        this.extenderSubsystem.goToState(RobotConstants.ExternderConstants.ExtenderState.CLOSE),
                        this.extenderSubsystem.goToState(RobotConstants.ExternderConstants.ExtenderState.OPEN)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        this.intakeSubsystem.toggleState(RobotConstants.IntakeConstants.IntakeState.TAKE)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        this.intakeSubsystem.toggleState(RobotConstants.IntakeConstants.IntakeState.DROP)
                );
    }

    @Override
    public void initialize() {
        this.mecanumDriveSubsystem.setDefaultCommand(new MecanumArcadeDriveCommand(this.mecanumDriveSubsystem, this.driverController));
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {

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

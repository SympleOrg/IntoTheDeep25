package org.firstinspires.ftc.teamcode.util.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.TurnToFLLCommand;
import org.firstinspires.ftc.teamcode.util.DataLogger;

@TeleOp(name = "Mecanum Tuner", group = "tune")
@Config
public class MecanumTunerOpMode extends CommandOpMode {
    public static double angle = 90;

    private GamepadEx gamepadEx;
    private MecanumDriveSubsystem mecanumDriveSubsystem;

    @Override
    public void initialize() {
        RobotPositionManager.init(hardwareMap);
        this.gamepadEx = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, (MultipleTelemetry) telemetry, new DataLogger("MecanumTuner"));

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new TurnToFLLCommand(this.mecanumDriveSubsystem, Rotation2d.fromDegrees(angle)));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Current Angle", RobotPositionManager.getInstance().getRelativeHeading());
        telemetry.update();
    }
}

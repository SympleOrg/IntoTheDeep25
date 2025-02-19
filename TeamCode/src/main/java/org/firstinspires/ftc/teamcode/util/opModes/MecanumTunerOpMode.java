package org.firstinspires.ftc.teamcode.util.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.RotateRobotByDegCommand;
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
                .whenPressed(new RotateRobotByDegCommand(this.mecanumDriveSubsystem, angle));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Required Angle", angle);
        telemetry.addData("Current Angle", RobotPositionManager.getInstance().getRelativeHeading());
        telemetry.addData("Error", angle - RobotPositionManager.getInstance().getRelativeHeading());
        telemetry.update();
    }
}

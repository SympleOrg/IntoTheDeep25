package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.tankDriveBase.TankArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.util.DataLogger;

@TeleOp(name = "Symple Tank Drive", group = "test")
public class SympleTankDrive extends CommandOpMode {
    private TankDriveSubsystem driveTrain;

    @Override
    public void initialize() {
        this.driveTrain = new TankDriveSubsystem(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()), new DataLogger("SympleTankDrive", false));
        this.driveTrain.setDefaultCommand(new TankArcadeDriveCommand(driveTrain, new GamepadEx(gamepad1)));
    }
}

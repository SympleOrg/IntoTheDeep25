package org.firstinspires.ftc.teamcode;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindLTV;
import com.pathplanner.lib.util.ReplanningConfig;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.SymplePathplannerUtils;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class AutoRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
    }

    @Override
    public void createKeyBindings() {

    }

    @Override
    public void initialize() {
        AutoBuilder.configureLTV(
                () -> SymplePathplannerUtils.toPose2d(this.mecanumDriveSubsystem.getLocalizer().getPose()),
                pose2d -> this.mecanumDriveSubsystem.getLocalizer().setPose(SymplePathplannerUtils.toPose2d(pose2d)),
                () -> SymplePathplannerUtils.toChassisSpeeds(this.mecanumDriveSubsystem.getLocalizer().getChassisSpeeds()),
                this.mecanumDriveSubsystem::moveFromWheelSpeed,
                0.02f,
                new ReplanningConfig(),
                () -> false,
//                this.mecanumDriveSubsystem
                null
        );
//        new PathfindLTV()
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
        public Builder() {
            this.logFilePrefix = "Auto";
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        @Override
        public AutoRobotController build() {
            return new AutoRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.logFilePrefix, this.logData);
        }
    }
}

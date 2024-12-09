package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

import java.io.IOException;
import java.nio.file.Path;

import top.symple.ftcpathplanner.auto.AutoBuilder;
import top.symple.ftcpathplanner.commands.PathPlannerAuto;
import top.symple.ftcpathplanner.config.ModuleConfig;
import top.symple.ftcpathplanner.config.RobotConfig;
import top.symple.ftcpathplanner.controllers.PPLTVController;
import top.symple.ftcpathplanner.util.PPFilesUtil;
import top.symple.ftcpathplanner.util.PPLogger;

public class AutoRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());

        PPLogger.setLogFunc((logType, s, stackTraceElements) -> {

            DataLogger.DataType dataType = DataLogger.DataType.INFO;
            if(logType == PPLogger.LogType.WARN) dataType = DataLogger.DataType.WARN;
            if(logType == PPLogger.LogType.ERROR) dataType = DataLogger.DataType.ERROR;
            this.getDataLogger().addData(dataType, s);
            this.getTelemetry().addData("PP", s);
            if(stackTraceElements != null) {
                StringBuilder builder = new StringBuilder();
                for (StackTraceElement stackTraceElement : stackTraceElements) {
                    builder.append(stackTraceElement).append("\n");
                }
                this.getDataLogger().addData(DataLogger.DataType.ERROR, builder.toString());
                this.getTelemetry().addData("PP", builder.toString());
            }
        });
    }

    @Override
    public void createKeyBindings() {

    }

    @Override
    public void initialize() {
        try {
            PPFilesUtil.syncAutoFromServer("192.168.43.155", 2525);
        } catch (IOException e) {
            this.getDataLogger().addData(DataLogger.DataType.WARN, "Failed to sync autos: ");
            this.getDataLogger().addData(DataLogger.DataType.WARN, e);
            this.getTelemetry().addData("PP", "Failed to sync autos");
        }

        try {
            RobotConfig robotConfig = new RobotConfig(
                    new ModuleConfig(0.096, 0.75, 1),
                    0.42
            );

            AutoBuilder.configure(
                    this.mecanumDriveSubsystem.getLocalizer()::getPose,
                    this.mecanumDriveSubsystem.getLocalizer()::setPose,
                    this.mecanumDriveSubsystem.getLocalizer()::getChassisSpeeds,
                    (chassisSpeeds, driveFeedforwards) -> this.mecanumDriveSubsystem.moveFromWheelSpeed(chassisSpeeds),
                    new PPLTVController(0.02f),
                    robotConfig,
                    () -> false,
                    this.mecanumDriveSubsystem
            );
            this.getDataLogger().addData(DataLogger.DataType.INFO, "Initialized Auto Settings");
        } catch (Exception e) {
            this.getDataLogger().addThrowable(e);
            throw new RuntimeException(e);
        }
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        try {
            new PathPlannerAuto("Test").schedule();
        } catch (Exception e) {
            this.getDataLogger().addThrowable(e);
            throw e;
        }
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

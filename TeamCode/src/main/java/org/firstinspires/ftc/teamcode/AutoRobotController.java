package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.AutoPath;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
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
        this.mecanumDriveSubsystem.setPose(new Pose2d()); // starting pose
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        AutoPath autoPath = Trajectories.getPath(Trajectories.Paths.PARK, mecanumDriveSubsystem);
        this.mecanumDriveSubsystem.setAutoPath(autoPath)
                .follow();
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

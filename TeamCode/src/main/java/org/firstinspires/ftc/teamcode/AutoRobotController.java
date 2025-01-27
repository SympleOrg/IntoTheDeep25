package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.StrafeInAngleMecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeJointSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.AutoPath;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class AutoRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final Pose2d startingPose;
    private final Trajectories.Paths path;
    private final IntakeJointSubsystem intakeJointSubsystem;

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData, Pose2d startingPose, Trajectories.Paths path) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), startingPose, this.getTelemetry(), this.getDataLogger());
        this.intakeJointSubsystem = new IntakeJointSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.startingPose = startingPose;
        this.path = path;
    }

    @Override
    public void createKeyBindings() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        AutoPath autoPath = Trajectories.getPath(this.path, this.mecanumDriveSubsystem, this.startingPose);
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
        private Pose2d pose2d;
        private Trajectories.Paths path;

        public Builder() {
            this.logFilePrefix = "Auto";
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        public Builder setStartingPose(Pose2d pose2d) {
            this.pose2d = pose2d;
            return this;
        }

        public Builder setPath(Trajectories.Paths path) {
            this.path = path;
            return this;
        }

        @Override
        public AutoRobotController build() {
            return new AutoRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.logFilePrefix, this.logData, this.pose2d, this.path);
        }
    }
}

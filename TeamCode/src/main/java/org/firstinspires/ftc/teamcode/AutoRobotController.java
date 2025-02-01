package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.StrafeInAngleMecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeJointSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.AutoPath;
import org.firstinspires.ftc.teamcode.trajectories.Dis1Autos;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class AutoRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final Pose2d startingPose;
    private final Trajectories.Paths path;
    private final IntakeJointSubsystem intakeJointSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;

    private final Dis1Autos dis1Autos;

    public static Pose2d END_POSE = new Pose2d();

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData, Pose2d startingPose, Trajectories.Paths path) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        END_POSE = startingPose;

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), startingPose, this.getTelemetry(), this.getDataLogger());
        this.intakeJointSubsystem = new IntakeJointSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.elevatorSubsystem = new ElevatorSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.clawSubsystem = new ClawSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.extenderSubsystem = new ExtenderSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.startingPose = startingPose;
        this.path = path;

        this.dis1Autos = new Dis1Autos(mecanumDriveSubsystem, this.elevatorSubsystem, this.clawSubsystem);
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
        new SequentialCommandGroup(
                this.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.CLOSED),
                this.clawSubsystem.moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                this.dis1Autos.chamberAuto()
        ).schedule();
//        new DriveDistanceDriveCommand(mecanumDriveSubsystem, 1).schedule();
//        new StrafeInAngleMecanumCommand(this.mecanumDriveSubsystem, 45, 0.5).schedule();
//        AutoPath autoPath = Trajectories.getPath(this.path, this.mecanumDriveSubsystem, this.startingPose);
//        this.mecanumDriveSubsystem.setAutoPath(autoPath)
//                .follow();
    }

    @Override
    public void run() {

    }

    @Override
    public void postRun() {
        END_POSE = this.mecanumDriveSubsystem.getPosition();
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

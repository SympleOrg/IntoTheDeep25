package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.StrafeInAngleMecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class AutoRobotController extends RobotControllerBase {
    private final SubsystemContainer subsystemContainer;
    private final Trajectories trajectory;

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData, Trajectories trajectory) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        this.subsystemContainer = new SubsystemContainer(getHardwareMap(), getTelemetry(), getDataLogger(), trajectory.getStartingPose());
        this.trajectory = trajectory;
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
        this.subsystemContainer.intakeJointSubsystem.moveToState(RobotConstants.IntakeJointConstants.JointState.CLOSED).schedule();
        this.trajectory.followTrajectory(this.subsystemContainer);
    }

    @Override
    public void run() {

    }

    @Override
    public void postRun() {

    }

    public static class Builder extends RobotControllerBase.Builder {
        private Trajectories trajectory;

        public Builder() {
            this.logFilePrefix = "Auto";
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        public Builder setTrajectory(Trajectories trajectory) {
            this.trajectory = trajectory;
            return this;
        }

        @Override
        public AutoRobotController build() {
            return new AutoRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.logFilePrefix, this.logData, trajectory);
        }
    }

    public static class SubsystemContainer {
        private final MecanumDrive mecanumDriveSubsystem;
        private final ClawSubsystem clawSubsystem;
        private final ScorerSubsystem scorerSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ElevatorSubsystem elevatorSubsystem;
        private final ExtenderSubsystem extenderSubsystem;
        private final IntakeJointSubsystem intakeJointSubsystem;

        private SubsystemContainer(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger, Pose2d pose2d) {
            this.mecanumDriveSubsystem = new MecanumDrive(hardwareMap, pose2d);
            this.clawSubsystem = new ClawSubsystem(hardwareMap, telemetry, dataLogger);
            this.scorerSubsystem = new ScorerSubsystem(hardwareMap, telemetry, dataLogger);
            this.intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, dataLogger);
            this.elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, dataLogger);
            this.extenderSubsystem = new ExtenderSubsystem(hardwareMap, telemetry, dataLogger);
            this.intakeJointSubsystem = new IntakeJointSubsystem(hardwareMap, telemetry, dataLogger);
        }

        public MecanumDrive getMecanumDriveSubsystem() {
            return mecanumDriveSubsystem;
        }

        public ClawSubsystem getClawSubsystem() {
            return clawSubsystem;
        }

        public ScorerSubsystem getScorerSubsystem() {
            return scorerSubsystem;
        }

        public IntakeSubsystem getIntakeSubsystem() {
            return intakeSubsystem;
        }

        public ElevatorSubsystem getElevatorSubsystem() {
            return elevatorSubsystem;
        }

        public ExtenderSubsystem getExtenderSubsystem() {
            return extenderSubsystem;
        }

        public IntakeJointSubsystem getIntakeJointSubsystem() {
            return intakeJointSubsystem;
        }
    }
}

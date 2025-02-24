package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.StrafeInAngleMecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.extender.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakePitchJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakejoint.IntakeRollJointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.scorer.ScorerSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class AutoRobotController extends RobotControllerBase {
    public static Rotation2d endRotation = new Rotation2d();
    private final SubsystemContainer subsystemContainer;
    private final Trajectories trajectory;

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData, Trajectories trajectory) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        this.trajectory = trajectory;
        this.subsystemContainer = new SubsystemContainer(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
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
        StrafeInAngleMecanumCommand.updateAngle(0);
        this.subsystemContainer.getClawSubsystem().moveToState(RobotConstants.ClawConstants.ClawState.CLOSE).schedule();
        this.subsystemContainer.getClawSubsystem().moveToState(RobotConstants.ClawConstants.ClawState.CLOSE).schedule();
        this.subsystemContainer.getIntakePitchJointSubsystem().moveToState(RobotConstants.IntakeJointConstants.JointPitchState.BASKET).schedule();
        this.trajectory.getTrajectory(subsystemContainer).schedule();

    }

    @Override
    public void run() {

    }

    @Override
    public void postRun() {
        endRotation = Rotation2d.fromDegrees(this.subsystemContainer.mecanumDriveSubsystem.getForwardDistanceDriven());
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
            return new AutoRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.logFilePrefix, this.logData, this.trajectory);
        }
    }

    public static class SubsystemContainer {
        private final ClawSubsystem clawSubsystem;
        private final MecanumDriveSubsystem mecanumDriveSubsystem;
        private final ElevatorSubsystem elevatorSubsystem;
        private final ExtenderSubsystem extenderSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final IntakePitchJointSubsystem intakePitchJointSubsystem;
        private final IntakeRollJointSubsystem intakeRollJointSubsystem;
        private final ScorerSubsystem scorerSubsystem;

        public SubsystemContainer(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
            this.clawSubsystem = new ClawSubsystem(hardwareMap, telemetry, dataLogger);
            this.mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry, dataLogger);
            this.elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, dataLogger);
            this.extenderSubsystem = new ExtenderSubsystem(hardwareMap, telemetry, dataLogger);
            this.intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, dataLogger);
            this.intakePitchJointSubsystem = new IntakePitchJointSubsystem(hardwareMap, telemetry, dataLogger);
            this.intakeRollJointSubsystem = new IntakeRollJointSubsystem(hardwareMap, telemetry, dataLogger);
            this.scorerSubsystem = new ScorerSubsystem(hardwareMap, telemetry, dataLogger);
        }

        public ClawSubsystem getClawSubsystem() {
            return clawSubsystem;
        }

        public MecanumDriveSubsystem getMecanumDriveSubsystem() {
            return mecanumDriveSubsystem;
        }

        public ElevatorSubsystem getElevatorSubsystem() {
            return elevatorSubsystem;
        }

        public ExtenderSubsystem getExtenderSubsystem() {
            return extenderSubsystem;
        }

        public IntakeSubsystem getIntakeSubsystem() {
            return intakeSubsystem;
        }

        public IntakePitchJointSubsystem getIntakePitchJointSubsystem() {
            return intakePitchJointSubsystem;
        }

        public IntakeRollJointSubsystem getIntakeRollJointSubsystem() {
            return intakeRollJointSubsystem;
        }

        public ScorerSubsystem getScorerSubsystem() {
            return scorerSubsystem;
        }
    }
}

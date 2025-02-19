package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.MecanumChassisUtils;

@Config
public class TurnToAngleWithDrive extends CommandBase {
    public static double Kp = 0.09;
    public static double Kd = 0.005;
    private static final double MAX_POWER = 0.8;

    private PIDController pController;
    private final double degToRotate;

    private final MecanumDriveSubsystem subsystem;

    public TurnToAngleWithDrive(MecanumDriveSubsystem subsystem, double degToRotate) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.degToRotate = degToRotate;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.pController = new PIDController(Kp, 0, Kd);
        this.pController.setTolerance(2);
        this.pController.setSetPoint(degToRotate + this.subsystem.getHeading());
        this.subsystem.getDataLogger().addData(DataLogger.DataType.INFO, "TurnToAngleWithDrive: " + "Rotating " + this.degToRotate + "deg");
    }

    @Override
    public void execute() {
        double rotationSpeed = this.pController.calculate(this.subsystem.getHeading());

        double power = Math.min(Math.max(rotationSpeed, -MAX_POWER), MAX_POWER);

        MecanumChassisUtils.MecanumWheelSpeeds speeds = MecanumChassisUtils.chassisSpeedToWheelSpeeds(new Vector2d(), rotationSpeed);

        MultipleTelemetry telemetry = this.subsystem.getTelemetry();
        telemetry.addData("power", power);
        telemetry.addData("Error", this.pController.getPositionError());
        telemetry.addData("rel heading", this.subsystem.getHeading() - this.pController.getSetPoint() - degToRotate);
        telemetry.addData("heading", this.subsystem.getHeading());
        telemetry.update();

        this.subsystem.moveMotors(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveSideMotors(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}

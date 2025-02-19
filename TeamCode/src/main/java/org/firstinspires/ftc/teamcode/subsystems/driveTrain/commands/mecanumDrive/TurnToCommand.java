package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.MecanumChassisUtils;

@Config
public class TurnToCommand extends CommandBase {
    public static double Kp = 0.08;
    public static double Ki = 0;
    public static double Kd = 0.001;

    private final MecanumDriveSubsystem driveTrain;
    private PIDController turnController;
    private Rotation2d targetAngle;
    private Rotation2d currentAngle;

    public TurnToCommand(MecanumDriveSubsystem driveTrain, Rotation2d targetAngle) {
        this.driveTrain = driveTrain;

        this.targetAngle = targetAngle;

        addRequirements(driveTrain);
    }


    @Override
    public void initialize() {
        currentAngle = Rotation2d.fromDegrees(driveTrain.getHeading());

        turnController = new PIDController(Kp, Ki, Kd);
        turnController.setSetPoint(-Math.abs(targetAngle.plus(currentAngle).getDegrees()));
        turnController.setTolerance(2);
    }

    @Override
    public void execute() {
        currentAngle = Rotation2d.fromDegrees(driveTrain.getHeading());

        double rotSpeed = turnController.calculate(currentAngle.getDegrees());
        rotSpeed = -Math.abs(rotSpeed); // Always make negetive.
        this.driveTrain.getTelemetry().addData("errorRotation", turnController.getPositionError());
        this.driveTrain.getTelemetry().addData("CurrentHeading", currentAngle.toString());
        MecanumChassisUtils.MecanumWheelSpeeds speeds =
                MecanumChassisUtils.chassisSpeedToWheelSpeeds(new Vector2d(), rotSpeed * 0.333);

        speeds.setBackLeft(0);

        driveTrain.moveMotors(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.moveSideMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetPoint();
    }
}

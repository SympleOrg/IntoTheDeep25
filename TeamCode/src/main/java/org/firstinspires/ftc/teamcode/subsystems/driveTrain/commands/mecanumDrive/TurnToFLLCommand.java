package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.MecanumChassisUtils;

@Config
public class TurnToFLLCommand extends CommandBase {
    public static double Kp = 0.08;
    public static double Ki = 0;
    public static double Kd = 0.001;

    private MecanumDriveSubsystem driveTrain;
    private PIDController turnController;
    private Rotation2d targetAngle;
    private Rotation2d currentAngle;

    public TurnToFLLCommand(MecanumDriveSubsystem driveTrain, Rotation2d targetAngle) {
        this.driveTrain = driveTrain;
        this.targetAngle = targetAngle;
        this.currentAngle = new Rotation2d();

        addRequirements(driveTrain);
    }


    @Override
    public void initialize() {
        turnController = new PIDController(Kp, Ki, Kd);
        this.currentAngle = Rotation2d.fromDegrees(Math.IEEEremainder(driveTrain.getHeading(), 360));
        turnController.setSetPoint(currentAngle.plus(targetAngle).getDegrees());
        turnController.setTolerance(2);

        this.driveTrain.setZeroPowerBehaviour(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, Motor.ZeroPowerBehavior.FLOAT);
        this.driveTrain.setZeroPowerBehaviour(MecanumDriveSubsystem.MotorNames.BACK_LEFT, Motor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void execute() {
        currentAngle = Rotation2d.fromDegrees(driveTrain.getHeading());
        double rotSpeed = turnController.calculate(Math.IEEEremainder(this.turnController.getSetPoint() - currentAngle.getDegrees(), 360));

        this.driveTrain.getTelemetry().addData("Rotation Err", turnController.getPositionError());
        this.driveTrain.getTelemetry().addData("Current Rotation", currentAngle.getDegrees() - turnController.getSetPoint() - targetAngle.getDegrees());
        this.driveTrain.getTelemetry().addData("Rotation Speed", rotSpeed);

        MecanumChassisUtils.MecanumWheelSpeeds speeds =
                MecanumChassisUtils.chassisSpeedToWheelSpeeds(new Vector2d(), rotSpeed);

        speeds.setFrontRight(0);
        speeds.setBackLeft(0);

        this.driveTrain.moveMotors(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.moveSideMotors(0, 0);
        this.driveTrain.setZeroPowerBehaviour(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, Motor.ZeroPowerBehavior.BRAKE);
        this.driveTrain.setZeroPowerBehaviour(MecanumDriveSubsystem.MotorNames.BACK_LEFT, Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetPoint();
    }
}

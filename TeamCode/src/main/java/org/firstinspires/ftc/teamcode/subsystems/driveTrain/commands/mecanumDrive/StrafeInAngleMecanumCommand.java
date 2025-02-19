package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.MecanumChassisUtils;

@Config
public class StrafeInAngleMecanumCommand extends CommandBase {
    public static double Kp = 1.8;
    public static double Kf = 1.5;
    public static double Ki = 1;

    public static double rotationKp = 0.02;

    private final double angle;
    private final double meters;

    private double STARTING_FORWARD_DIST = 0;
    private double STARTING_SIDE_DIST = 0;
    private PIDFController pidfController;

    private PIDController rotationController;

    private final MecanumDriveSubsystem subsystem;

    public StrafeInAngleMecanumCommand(MecanumDriveSubsystem subsystem, double angle, double meters) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.angle = angle + 90;
        this.meters = meters;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.STARTING_FORWARD_DIST = this.subsystem.getForwardDistanceDriven();
        this.STARTING_SIDE_DIST = this.subsystem.getSideDistanceDriven();

        this.pidfController = new PIDFController(Kp, Ki, 0, Kf);
        this.pidfController.setTolerance(0.0185);
        this.pidfController.setSetPoint(this.meters);

        this.rotationController = new PIDController(rotationKp, 0, 0);
        this.rotationController.setSetPoint(this.subsystem.getHeading());
        this.rotationController.setTolerance(0);
    }

    @Override
    public void execute() {
        double hSpeed = Math.cos(Math.toRadians(angle));
        double vSpeed = Math.sin(Math.toRadians(angle));
        Vector2d vector2d = new Vector2d(hSpeed, vSpeed);

        double rotationSpeed = this.rotationController.calculate(this.subsystem.getHeading());

        double forwardDistanceMoved = this.subsystem.getForwardDistanceDriven() - this.STARTING_FORWARD_DIST;
        double sideDistanceMoved = this.subsystem.getSideDistanceDriven() - this.STARTING_SIDE_DIST;

        double currentDist = Math.hypot(forwardDistanceMoved, sideDistanceMoved); // => √x*x + y*y
        double powerMultiplier = this.pidfController.calculate(currentDist);

        MecanumChassisUtils.MecanumWheelSpeeds mecanumWheelSpeeds = MecanumChassisUtils.chassisSpeedToWheelSpeeds(vector2d, rotationSpeed)
                .mul(powerMultiplier);

        this.subsystem.moveMotors(mecanumWheelSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_LEFT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_LEFT, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pidfController.atSetPoint();
    }
}

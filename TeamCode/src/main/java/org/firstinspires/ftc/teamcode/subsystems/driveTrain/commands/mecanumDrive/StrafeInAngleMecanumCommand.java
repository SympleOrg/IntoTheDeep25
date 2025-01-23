package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.MecanumChassisUtils;

public class StrafeInAngleMecanumCommand extends CommandBase {
    private final double angle;
    private final double meters;

    private double STARTING_FORWARD_DIST = 0;
    private double STARTING_SIDE_DIST = 0;
    private PController pController;

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

        this.pController = new PController(0.8);
        this.pController.setTolerance(0.02);
        this.pController.setSetPoint(this.meters);
    }

    @Override
    public void execute() {
        double hSpeed = Math.cos(Math.toRadians(angle)) * meters;
        double vSpeed = Math.sin(Math.toRadians(angle)) * meters;
        Vector2d vector2d = new Vector2d(hSpeed, vSpeed);

        double forwardDistanceMoved = this.subsystem.getForwardDistanceDriven() - this.STARTING_FORWARD_DIST;
        double sideDistanceMoved = this.subsystem.getSideDistanceDriven() - this.STARTING_SIDE_DIST;

        double currentDist = Math.hypot(forwardDistanceMoved, sideDistanceMoved); // => √x*x + y*y
        double powerMultiplier = this.pController.calculate(currentDist);

        MecanumChassisUtils.MecanumWheelSpeeds mecanumWheelSpeeds = MecanumChassisUtils.chassisSpeedToWheelSpeeds(vector2d, 0)
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
        return this.pController.atSetPoint();
    }
}

package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.MecanumChassisUtils;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem.MecanumChassisWheelsSet.MotorNames;


public class StrafeInAngleMecanumCommand extends CommandBase {
    private final double angle;
    private final double meters;

    private Pose2d STARTING_POSE;
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
        this.STARTING_POSE = this.subsystem.getPosition();

        this.pController = new PController(1);
        this.pController.setTolerance(0.02);
        this.pController.setSetPoint(this.meters);
    }

    @Override
    public void execute() {
        double hSpeed = Math.cos(Math.toRadians(angle));
        double vSpeed = Math.sin(Math.toRadians(angle));
        Vector2d vector2d = new Vector2d(hSpeed, vSpeed);

        Transform2d distMoved = this.subsystem.getPosition().minus(this.STARTING_POSE);

        double powerMultiplier = this.pController.calculate(distMoved.getTranslation().getNorm());

        MecanumChassisUtils.MecanumWheelSpeeds mecanumWheelSpeeds = MecanumChassisUtils.chassisSpeedToWheelSpeeds(vector2d, 0)
                .mul(powerMultiplier);

        this.subsystem.moveMotors(mecanumWheelSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveMotor(MotorNames.FRONT_RIGHT, 0);
        this.subsystem.moveMotor(MotorNames.BACK_RIGHT, 0);
        this.subsystem.moveMotor(MotorNames.FRONT_LEFT, 0);
        this.subsystem.moveMotor(MotorNames.BACK_LEFT, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}

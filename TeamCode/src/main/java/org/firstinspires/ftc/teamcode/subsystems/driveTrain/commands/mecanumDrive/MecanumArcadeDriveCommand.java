package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;

public class MecanumArcadeDriveCommand extends CommandBase {
    private final GamepadEx gamepad;

    private final MecanumDriveSubsystem subsystem;

    public MecanumArcadeDriveCommand(MecanumDriveSubsystem subsystem, GamepadEx gamepad) {
        this.subsystem = subsystem;
        this.gamepad = gamepad;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double vSpeed = this.gamepad.getLeftY();
        double hSpeed = -this.gamepad.getLeftX();
        double rotationSpeed = -this.gamepad.getRightX();

        Vector2d vector = new Vector2d(hSpeed, vSpeed);
        vector = vector.rotateBy(this.subsystem.getHeading());

        double[] speeds = new double[4];
        speeds[0] = Math.sin(vector.angle() + Math.PI / 4) + rotationSpeed; // front left
        speeds[1] = Math.sin(vector.angle() - Math.PI / 4) - rotationSpeed; // front right
        speeds[2] = Math.sin(vector.angle() - Math.PI / 4) + rotationSpeed; // back left
        speeds[3] = Math.sin(vector.angle() + Math.PI / 4) - rotationSpeed; // back right

        normalize(speeds, vector.magnitude());

        speeds[0] += rotationSpeed;
        speeds[1] -= rotationSpeed;
        speeds[2] += rotationSpeed;
        speeds[3] -= rotationSpeed;

        normalize(speeds);

        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_LEFT, speeds[0]);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, speeds[1]);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_LEFT, speeds[2]);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_RIGHT, speeds[3]);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_LEFT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_LEFT, 0);
        super.end(interrupted);
    }

    private void normalize(double[] wheelSpeeds, double mag) {
        double maxSpeed = this.getMaxSpeeds(wheelSpeeds);

        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxSpeed) * mag;
        }
    }

    private void normalize(double[] wheelSpeeds) {
        double maxSpeed = this.getMaxSpeeds(wheelSpeeds);

        if(maxSpeed > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxSpeed;
            }
        }
    }

    private double getMaxSpeeds(double[] wheelSpeeds) {
        double maxSpeed = 0;
        for(double speed : wheelSpeeds) {
            maxSpeed = Math.max(maxSpeed, Math.abs(speed));
        }
        return maxSpeed;
    }
}

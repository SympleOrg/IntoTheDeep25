package org.firstinspires.ftc.teamcode.util.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.SympleServo;

@Config
@TeleOp(name = "Servo Tuner", group = "tune")
public class ServoTunerOpMode extends CommandOpMode {
    private final ServoMap servoName = ServoMap.INTAKE_X_JOINT;
    public static double angle = 0;

    private GamepadEx gamepadEx;
    private SympleServo servo;
    @Override
    public void initialize() {
        this.gamepadEx = new GamepadEx(gamepad2);
        servo = new SympleServo(hardwareMap, servoName.getId(), 0, 300, AngleUnit.DEGREES);
    }

    @Override
    public void run() {
        super.run();
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(new InstantCommand(() -> servo.turnToAngle(angle)));
        telemetry.addData("angle", angle);
        telemetry.update();
    }
}

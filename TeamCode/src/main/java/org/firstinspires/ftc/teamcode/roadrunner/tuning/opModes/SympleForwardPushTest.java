package org.firstinspires.ftc.teamcode.roadrunner.tuning.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

public class SympleForwardPushTest extends LinearOpMode {
    private DriveViewFactory dvf;

    public SympleForwardPushTest(DriveViewFactory dvf) {
        super();
        this.dvf = dvf;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = dvf.make(hardwareMap);

        for (DcMotorEx m : view.getMotors()) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        waitForStart();

        double initAvgPos = avgPos(view.getForwardEncs());
        MultipleTelemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (opModeIsActive()) {
            t.addData("ticks traveled", avgPos(view.getForwardEncs()) - initAvgPos);
            t.update();
        }
    }

    private double avgPos(List<RawEncoder> encoders) {
        double d = 0;
        for (RawEncoder encoder : encoders) {
            d += encoder.getPositionAndVelocity().position;
        }
        return d;
    }
}

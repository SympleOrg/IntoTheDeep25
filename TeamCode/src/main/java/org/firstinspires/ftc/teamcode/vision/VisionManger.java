package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionManger {
    private final OpenCvCamera camera;
    private final DataLogger dataLogger;

    public VisionManger(HardwareMap hardwareMap, DataLogger dataLogger) {
        this.dataLogger = dataLogger;

        this.dataLogger.addData(DataLogger.DataType.INFO, "[Vision] Trying to get camera.");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, SensorMap.WEBCAM.getId()), cameraMonitorViewId);
        this.dataLogger.addData(DataLogger.DataType.INFO, "[Vision] Camera found!");

        this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                VisionManger.this.camera.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE);
                dataLogger.addData(DataLogger.DataType.INFO, "[Vision] Starting to stream camera.");
            }

            @Override
            public void onError(int errorCode) {
                dataLogger.addData(DataLogger.DataType.ERROR, "[Vision] Error while trying to open the camera, Error code: " + errorCode);
            }
        });

        this.dataLogger.addData(DataLogger.DataType.INFO, "[Vision] Starting to stream to ftc dashboard.");
        FtcDashboard.getInstance().startCameraStream(this.camera, 30);
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        this.dataLogger.addData(DataLogger.DataType.INFO, String.format("[Vision] Setting vision pipeline to '%s.'", pipeline.getClass().getSimpleName()));
        this.camera.setPipeline(pipeline);
    }
}

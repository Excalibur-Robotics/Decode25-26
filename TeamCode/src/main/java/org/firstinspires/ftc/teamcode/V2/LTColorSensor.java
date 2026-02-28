package org.firstinspires.ftc.teamcode.V2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "CST")
public class LTColorSensor extends OpMode {
    OpenCvCamera LT; //Logitech
    LTPipeline pipeline;

    @Override
    public void init(){
        int LT_ID=hardwareMap.appContext.getResources().getIdentifier("LT_ID", "id",
                hardwareMap.appContext.getPackageName());

        LT= OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"LT")
                ,LT_ID); //Webcam object

        pipeline = new LTPipeline();
        LT.setPipeline(pipeline);
        LT.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
             @Override
             public void onOpened() {
                 LT.startStreaming(480, 640);
                 /*Adjust height and width of camera view here*/
             }

             @Override
             public void onError(int errorCode) {
                 // Does nothing if the cam doesn't open
             }
        }
        );

    }
    public void loop(){
        telemetry.addData("GreenPixels: ", pipeline.GreenPixels);
        telemetry.addData("PurplePixels: ", pipeline.PurplePixels);
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(LT, 0);
    }
}

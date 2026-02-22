package org.firstinspires.ftc.teamcode.V2.Subsystems;

import static java.lang.Math.abs;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.V2.LHV2PID;
import org.firstinspires.ftc.teamcode.V2.LTPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;

/*
This is the subsystem for the spindexer of V1. It includes a motor to turn the
spindexer and a color sensor.
spindexer motor: can rotate spindexer one slot and switch modes*
color sensor: detects when an artifact is in slot 0 and its color

This subsystem also contains an arraylist to store the state of the spindexer,
which is modified when the spindexer rotates and when an artifact is intaked or
outtaked. It also has a variable containing the number of artifacts currently
in the spindexer

*Because the outtake and intake positions don't line up on the robot, the
spindexer can be rotate 60 degrees to switch between intake and outtake mode.
When this happens, the state of the indexer arraylist doesn't change.
 */

@Config
public class SpindexerSubsystem extends SubsystemBase {
    public DcMotorEx spindexMotor;
    public OpenCvCamera LT;
    LTPipeline pipeline;

    private ArrayList<String> indexer;
    private int numArtifacts;
    private boolean OuttakeMode;

    // spindexer pid constants
    public static double kP = 0.0006;
    public static double kI = 0;
    public static double kD = 0.008;
    public static int tolerance = 35;
    public static int velocityTolerance = 15;
    public final int ticksPerRev = 8192; // cpr of bore encoder

    public LHV2PID PID;
    private double TP; // target position in ticks

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    public SpindexerSubsystem(HardwareMap hwMap) {
        spindexMotor = hwMap.get(DcMotorEx.class, "Bore");
        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int CP = spindexMotor.getCurrentPosition();
        int ticks = 120 * ticksPerRev / 360;
        if(CP != 0)
            TP = (abs(CP) + ticks/2) / ticks * ticks * CP / abs(CP);
        else
            TP = 0;

        PID = new LHV2PID(kP, kI, kD);
        timer.reset();

        indexer = new ArrayList<String>();
        indexer.add("purple");
        indexer.add("green");
        indexer.add("purple");
        numArtifacts = 3; // start with 3 preloads
        OuttakeMode = true; // start spindexer in outtake mode

        // camera initialization
        int LT_ID=hwMap.appContext.getResources().getIdentifier("LT_ID", "id",
                hwMap.appContext.getPackageName());
        LT = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class,"LT")
                ,LT_ID); //Webcam object
        pipeline = new LTPipeline();
        LT.setPipeline(pipeline);
        LT.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             LT.startStreaming(640, 480);
                                             /*Adjust height and width of camera view here*/
                                         }

                                         @Override
                                         public void onError(int errorCode) {
                                             // Does nothing if the cam doesn't open
                                         }
                                     }
        );
    }

    @Override
    public void periodic() {
        //powerSpindexer();
    }

    // power spindexer based on PID
    public void powerSpindexer() {
        double CP = spindexMotor.getCurrentPosition();
        if(timer.milliseconds()>15) {
            CP = spindexMotor.getCurrentPosition();
            double MotorPower = -PID.Calculate(TP, CP);
            spindexMotor.setPower(MotorPower);
            Log.i("spindexer", String.valueOf(MotorPower));
            timer.reset();
        }
    }

    // rotate spindexer clockwise one slot (120 degrees)
    public void rotateCW() {
        TP = TP - 120.0 / 360 * ticksPerRev;
        indexer.add(indexer.remove(0));
    }
    // rotate spindexer counter-clockwise one slot
    public void rotateCCW() {
        TP = TP + 120.0 / 360 * ticksPerRev;
        indexer.add(0, indexer.remove(2));
    }
    // rotate 60 degrees to outtake mode
    public void setToOuttakeMode() {
        TP = TP - 60.0 / 360 * ticksPerRev;
        OuttakeMode = true;
    }
    // rotate back 60 degrees to intake mode
    public void setToIntakeMode() {
        TP = TP + 60.0 / 360 * ticksPerRev;
        OuttakeMode = false;
    }

    public void resetSpindexEncoder() {
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setSpindexMotorPower(double power) {
        spindexMotor.setPower(power);
    }

    // returns angle spindexer has rotated in degrees
    public double getSpindexerAngle() {
        return (double) spindexMotor.getCurrentPosition() / ticksPerRev * 360;
    }
    public double getTargetAngle() {
        return TP / ticksPerRev * 360;
    }

    public double getSpindexerPower() {
        return spindexMotor.getPower();
    }

    public double getTP() {
        return TP;
    }

    public boolean inOuttakeMode() {
        return OuttakeMode;
    }

    // get the arraylist of the artifacts
    public ArrayList<String> getIndexerState() {
        return indexer;
    }

    // get the number of artifacts in the spindexer
    public int getNumArtifacts() {
        return numArtifacts;
    }

    // add an artifact to slot 0 when intaked
    public void addArtifact(String color) {
        if(indexer.get(0).equals("empty")) {
            indexer.set(0, color);
            numArtifacts++;
        }
    }
    // remove an artifact from slot 2 when outtaked
    public void removeArtifact() {
        if(!indexer.get(2).equals("empty")) {
            indexer.set(2, "empty");
            numArtifacts--;
        }
    }

    public void setIndexerState(String[] state) {
        int num = 0;
        for(int i = 0; i < 3; i++) {
            indexer.set(i, state[i]);
            if(!state[i].equals("empty"))
                num++;
        }
        numArtifacts = num;
    }

    public double getGreenPixels() {
        return pipeline.GreenPixels;
    }
    public double getPurplePixels() {
        return pipeline.PurplePixels;
    }

    // get the color the color sensor currently sees.
    public String getColor() {
        String color = "empty";
        if(pipeline.GreenPixels > 90000) {
            color = "green";
        }
        else if(pipeline.PurplePixels > 90000) {
            color = "purple";
        }

        return color;
    }

    public boolean detectsArtifact() {
        return pipeline.GreenPixels > 90000 || pipeline.PurplePixels > 90000;
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="odotest")
public class Odom extends OpMode{

    public GoBildaPinpointDriver odo;
    double OldTime=0;
    double xOffset=-150;
    double yOffset=-135;
    @Override
    public void init() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("X", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("DeviceV", odo.getDeviceVersion());
        telemetry.addData("DeviceS", odo.getYawScalar());
    }

        @Override
        public void loop(){
            odo.update();
            double NewTime= getRuntime();
            double LoopTime= NewTime-OldTime;
            double freq= 1/LoopTime; //This is the frequency of the odometry
            OldTime=NewTime;
            Pose2D pos= odo.getPosition();
            /*Double velx = odo.getVelX(DistanceUnit.MM);
            Double vely=odo.getVelY(DistanceUnit.MM);
           Double vel = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            String velocity=String.format(Locale.US,
                    "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                    velx, vely,
                    vel);
            telemetry.addData("Velocity", velocity);*/
            telemetry.addData("Position", pos);
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pin Freq", odo.getFrequency());
            telemetry.addData("REV Hub Freq", freq);
            
        }

    }



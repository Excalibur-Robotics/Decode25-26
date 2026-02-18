package org.firstinspires.ftc.teamcode.V2.Autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

/*
This is the main OpMode for our autonomous programs.
It creates and autonomous routine object (CloseAuto or FarAuto) which contains
information about the specific routine. You can also set the team color for
each routine. It also creates the follower, which is what makes the robot
actually follow the path.
*/

@Autonomous
public class AutoOpMode extends OpMode {
    // declare the follower
    private Follower follower;
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;

    // declare an autonomous routine
    AutoRoutine routine;
    boolean onRedTeam = true;
    boolean startingClose = false;

    @Override
    public void init() {
        // initialize follower
        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        outtake.setHood(outtake.getHoodFar());
        spindexer.resetSpindexEncoder();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Choose Team Color","press square if blue team, circle if red team");
        telemetry.addData("Team Color", "");
        telemetry.addLine();
        telemetry.addData("Choose starting position", "press triangle if close, X if far (from goal)");
        telemetry.addData("Starting Position", "");
        if(gamepad1.x) {
            onRedTeam = false;
            //outtake.setTeam(false);
            telemetry.addData("Team Color", "BLUE");
        }
        if(gamepad1.b) {
            onRedTeam = true;
            //outtake.setTeam(true);
            telemetry.addData("Team Color", "RED");
        }
        if(gamepad1.y) {
            startingClose = true;
            telemetry.addData("Starting Position", "CLOSE");
        }
        if(gamepad1.a) {
            startingClose = false;
            telemetry.addData("Starting Position", "FAR");
        }
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        routine = new AutoRoutine(follower, intake, spindexer, outtake, onRedTeam, startingClose);
        follower.setStartingPose(routine.getPoses().get(0));
    }

    @Override
    public void loop() {
        // This is the main loop, which determines the current path
        // and makes the robot follow it
        follower.update();
        routine.autoPathUpdate(follower);

        CommandScheduler.getInstance().run();
        outtake.calculateTurretLL(outtake.getTX());
        outtake.calculateLaunch();
        spindexer.powerSpindexer();

        /*try {
            Thread.sleep(5);
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        spindexer.powerSpindexer();
        try {
            Thread.sleep(5);
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }

         */

        // telemetry for debugging: current path and robot pose
        telemetry.addData("team color", onRedTeam ? "RED" : "BLUE");
        telemetry.addData("close or far:", startingClose ? "CLOSE" : "FAR");
        telemetry.addData("path state", routine.getPathState());
        telemetry.addData("follower is busy?", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("spindexer position", spindexer.getSpindexerAngle());
        telemetry.addData("spindexer target position", spindexer.getTargetAngle());
        telemetry.addData("spindexer power", spindexer.getSpindexerPower());
        ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("spindexer" , spindexer.inOuttakeMode() ? "  " +
                indexer.get(2).charAt(0) : " " + indexer.get(2).charAt(0) + " " + indexer.get(1).charAt(0));
        telemetry.addData("state        ", spindexer.inOuttakeMode() ? " " + indexer.get(0).charAt(0)
                + " " + indexer.get(1).charAt(0) : "   " + indexer.get(0).charAt(0));
        telemetry.addData("# artifacts", spindexer.getNumArtifacts());
        telemetry.addData("team color", onRedTeam ? "RED" : "BLUE");
        telemetry.addData("tx", outtake.getTX());
        telemetry.addData("ta", outtake.getTA());
        telemetry.update();
    }
}

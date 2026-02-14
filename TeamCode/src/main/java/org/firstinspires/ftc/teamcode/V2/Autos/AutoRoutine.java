package org.firstinspires.ftc.teamcode.V2.Autos;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

import java.util.ArrayList;

public class AutoRoutine {
    private int pathState;
    private boolean closeAuto;

    ArrayList<Pose> poses = new ArrayList<Pose>();
    ArrayList<PathChain> paths = new ArrayList<PathChain>();

    public IntakeSubsystem intake;
    public SpindexerSubsystem spindexer;
    public OuttakeSubsystem outtake;

    Trigger scheduleFlywheel;
    Trigger scheduleIntake;
    Trigger scheduleShoot;

    private int motif; // equals position of green ball
    private ElapsedTime pathTimer;

    AutoRoutine(Follower follower, IntakeSubsystem intake, SpindexerSubsystem spindexer,
                OuttakeSubsystem outtake, boolean redTeam, boolean startClose) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.outtake = outtake;
        pathTimer = new ElapsedTime();

        if(startClose) {
            poses.add(new Pose(125, 123, Math.toRadians(36))); // start pose
            poses.add(new Pose(95, 101, Math.toRadians(44))); // shoot pose
            poses.add(new Pose(100.0, 84.0, 0)); // before pickup
            poses.add(new Pose(126.0, 84.0, 0)); // after pickup

            if(!redTeam) {
                poses.replaceAll(item -> new Pose(144 - item.getX(), item.getY(),
                        Math.PI-item.getHeading()));
            }

            // set paths, same for both colors
            paths.add(follower.pathBuilder() // go to first shoot
                    .addPath(new BezierLine(poses.get(0), poses.get(1)))
                    .setLinearHeadingInterpolation(poses.get(0).getHeading(), poses.get(1).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // go to pick up
                    .addPath(new BezierLine(poses.get(1), poses.get(2)))
                    .setLinearHeadingInterpolation(poses.get(1).getHeading(), poses.get(2).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // intake balls
                    .addPath(new BezierLine(poses.get(2), poses.get(3)))
                    .setConstantHeadingInterpolation(poses.get(2).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // go to second shoot
                    .addPath(new BezierLine(poses.get(3), poses.get(1)))
                    .setLinearHeadingInterpolation(poses.get(3).getHeading(), poses.get(1).getHeading())
                    .build());

            // set triggers for commands
            scheduleFlywheel = new Trigger(() -> pathState == 0 || pathState == 1 || pathState == 4);
            scheduleIntake = new Trigger(() -> pathState == 3);
            scheduleShoot = new Trigger(() -> (pathState == 1 || pathState == 4) && !follower.isBusy());

            // bind triggers to commands
            scheduleFlywheel.whileActiveContinuous(new ActivateFlywheel(outtake));
            scheduleIntake.whileActiveOnce(new IntakeCommand(intake, spindexer));
            scheduleShoot.whileActiveContinuous(new ShootArtifact(outtake, spindexer));
        }
        else { // if starting far
            poses.add(new Pose(87, 9, Math.PI/2)); // start pose
            poses.add(new Pose(87, 36, Math.PI/2)); // off launch line
            poses.add(new Pose(126, 36, 0)); // after pickup
            poses.add(new Pose(87, 12, Math.PI/2)); // shoot pose

            if(!redTeam) {
                poses.replaceAll(item -> new Pose(144 - item.getX(), item.getY(),
                        Math.PI-item.getHeading()));
            }

            // set paths, same for both colors
            paths.add(follower.pathBuilder() // go to pick up
                    .addPath(new BezierLine(poses.get(0), poses.get(1)))
                    .setLinearHeadingInterpolation(poses.get(0).getHeading(), poses.get(1).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // intake balls
                    .addPath(new BezierLine(poses.get(1), poses.get(2)))
                    .setConstantHeadingInterpolation(poses.get(1).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // go to second shoot
                    .addPath(new BezierLine(poses.get(2), poses.get(3)))
                    .setLinearHeadingInterpolation(poses.get(2).getHeading(), poses.get(3).getHeading())
                    .build());

            // set triggers for commands
            scheduleFlywheel = new Trigger(() -> pathState == 0 || pathState == 3);
            scheduleIntake = new Trigger(() -> pathState == 2);
            scheduleShoot = new Trigger(() -> (pathState == 0) && !follower.isBusy()
                                                && outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30);

            // bind triggers to commands
            scheduleFlywheel.whileActiveContinuous(new ActivateFlywheel(outtake));
            scheduleIntake.whileActiveContinuous(new IntakeCommand(intake, spindexer));
            scheduleShoot.whileActiveContinuous(new ShootArtifact(outtake, spindexer), false);
        }

        pathState = 0;
        closeAuto = startClose;

        outtake.setLLPipeline(0);
        String[] colors = {"green", "purple", "purple"};
        spindexer.setIndexerState(colors);
    }

    // set the path state
    public void setPathState(int state) {
        pathState = state;
        pathTimer.reset();
    }
    // get the current path state for debugging
    public int getPathState() {
        return pathState;
    }
    public ArrayList<Pose> getPoses() {
        return poses;
    }
    public ArrayList<PathChain> getPaths() {
        return paths;
    }

    public void autoPathUpdate(Follower follower) {
        if(closeAuto) {
            switch (pathState) {
                case 0:
                    follower.followPath(paths.get(0));
                    setPathState(1);
                    break;
                case 1:
                    if(follower.isBusy()) {
                        //outtake.calculateTurret(-90 - outtake.getTurretPos());
                        if(outtake.getApriltagID() > 20 && outtake.getApriltagID() < 24) {
                            motif = outtake.getApriltagID() - 21;
                        }
                    }
                    else {
                        //outtake.setLLPipeline(2);
                        //outtake.calculateTurret(outtake.getTX());
                        spindexer.powerSpindexer();
                        if(spindexer.getNumArtifacts() == 0){
                            follower.followPath(paths.get(1));
                            setPathState(2);
                        }
                    }
                    break;
                case 2:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.get(2));
                        setPathState(3);
                    }
                    break;
                case 3:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.get(3));
                        setPathState(4);
                    }
                    break;
                case 4:
                    if(!follower.isBusy()) {
                        if(spindexer.getNumArtifacts() == 0){
                            setPathState(-1);
                        }
                    }
                    break;
            }
        }
        else {
            switch (pathState) {
                case 0:
                    //spindexer.powerSpindexer();
                    if(spindexer.getNumArtifacts() < 3) {
                        follower.followPath(paths.get(0));
                        setPathState(1);
                    }
                    break;
                case 1:
                    if(!follower.isBusy()) {
                        //follower.followPath(paths.get(1));
                        setPathState(-1);
                    }
                    break;
                case 2:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.get(2));
                        setPathState(3);
                    }
                    break;
                case 3:
                    if(!follower.isBusy()) {
                        if(spindexer.getNumArtifacts() == 0) {
                            setPathState(-1);
                        }
                    }
                    break;
            }
        }
    }
}

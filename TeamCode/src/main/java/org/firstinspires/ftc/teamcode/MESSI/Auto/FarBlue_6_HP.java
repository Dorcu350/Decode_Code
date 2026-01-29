package org.firstinspires.ftc.teamcode.MESSI.Auto;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Hang;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous(group = "Far")
public class FarBlue_6_HP extends OpMode {
    Shooter shooter;
    Intake intake;
    Hang hang;
    Sorter sorter;
    Sensors sensors;
    Globals globals;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(8, -6, 0.5);
    private final Pose firstPrePose = new Pose(8, 43, -1);
    private final Pose firstIntake = new Pose(3, 43, -1);
    private final Pose reAlignPose = new Pose(0, 39 , -1.56);
    private final Pose finessePose = new Pose(0, 40, -2);
    private final Pose finishFinessePose = new Pose(0, 45, -1.56);
    private Path scorePreload, grabFirst, finishFirst, reAlign, goBack, goBackIn, finishFinesse,scoreFirst;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabFirst = new Path(new BezierLine(scorePose, firstPrePose));
        grabFirst.setLinearHeadingInterpolation(scorePose.getHeading(), firstPrePose.getHeading());

        finishFirst = new Path(new BezierLine(firstPrePose, firstIntake));
        finishFirst.setConstantHeadingInterpolation(firstIntake.getHeading());

        reAlign = new Path(new BezierLine(firstIntake, reAlignPose));
        reAlign.setLinearHeadingInterpolation(firstIntake.getHeading(), reAlignPose.getHeading());

        goBackIn = new Path(new BezierLine(reAlignPose, finessePose));
        goBackIn.setLinearHeadingInterpolation(reAlignPose.getHeading(), finessePose.getHeading());

        goBack = new Path(new BezierLine(finessePose, reAlignPose));
        goBack.setLinearHeadingInterpolation(finessePose.getHeading(), reAlignPose.getHeading());

        finishFinesse = new Path(new BezierLine(reAlignPose, finishFinessePose));
        finishFinesse.setLinearHeadingInterpolation(reAlignPose.getHeading(), finishFinessePose.getHeading());

        scoreFirst = new Path(new BezierLine(finishFinessePose, scorePose));
        scoreFirst.setConstantHeadingInterpolation(scorePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Shooter.check_align = false;
                follower.followPath(scorePreload);
                shooter.state = Shooter.State.IDLE;
                setPathState(1);
                break;
            case 1:
                if (littleError(scorePose.getX(), scorePose.getY()))
                    shooter.state = Shooter.State.SHOOT;

                if (shooter.state == Shooter.State.SHOOT && pathTimer.getElapsedTimeSeconds() > 4) {
                    shooter.state = Shooter.State.STOPPED;
                    follower.followPath(grabFirst);
                    setPathState(2);
                }
                break;
            case 2:
                if (littleError(firstPrePose.getX(), firstPrePose.getY())) {
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(finishFirst);
                    follower.setMaxPower(0.5);
                    setPathState(3);
                }
                break;
            case 3:
                if(littleError(firstIntake.getX(), firstIntake.getY())) {
                    follower.followPath(reAlign);
                    follower.setMaxPower(1);
                    setPathState(4);
                }
                break;
            case 4:
                if(littleError(reAlignPose.getX(), reAlignPose.getY())) {
                    follower.followPath(goBackIn);
                    setPathState(5);
                }
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(goBack);
                    setPathState(6);
                }
                break;
            case 6:
                if(littleError(reAlignPose.getX(), reAlignPose.getY())) {
                    follower.followPath(finishFinesse);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.STOP;
                    follower.followPath(scoreFirst);
                    setPathState(8);
                }
                break;
            case 8:
                if (littleError(scorePose.getX(), scorePose.getY()))
                    shooter.state = Shooter.State.SHOOT;

                if (shooter.state == Shooter.State.SHOOT && pathTimer.getElapsedTimeSeconds() > 4) {
                    shooter.state = Shooter.State.STOPPED;
                    follower.followPath(grabFirst);
                    setPathState(9);
                }
                break;
        }
    }

    @Override
    public void loop() {
        shooter.update_shooter();
        intake.update_intake(gamepad1);
        hang.update();
        follower.update();

        if(sensors.check_for_shooting())
            gamepad1.rumble(1);

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("no error", littleError(scorePose.getX(), scorePose.getY()));
        telemetry.addData("vel ", shooter.motor_shooter.getVelocity());
        telemetry.addData(" error val ", Shooter.error);
        telemetry.addData("vede senzoor ", sensors.check_for_shooting());
        telemetry.addData("on target ", sensors.onTarget());
        telemetry.update();
    }

    @Override
    public void init() {
        sensors = new Sensors(hardwareMap);
        hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        sorter = new Sorter(hardwareMap);
        globals = new Globals();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("dumnezeu sa ierte victimele mele");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public boolean littleError(double targetX, double targetY) {
        return abs(follower.getPose().getX() - targetX) <= 1 && abs(follower.getPose().getY() - targetY) <= 1;
    }
}
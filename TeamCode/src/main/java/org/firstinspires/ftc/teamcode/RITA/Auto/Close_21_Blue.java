package org.firstinspires.ftc.teamcode.RITA.Auto;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RITA.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RITA.Utils.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "CLOSE")
public class Close_21_Blue extends OpMode {
    Shooter shooter;
    Intake intake;
    Sensors sensors;
    Turret turret;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(15.5, 111, Math.toRadians(180));
    private final Pose scorePose = new Pose(56, 86, Math.toRadians(135));
    private final Pose intakeSecondLine = new Pose(47, 59, Math.toRadians(180));
    private final Pose finishSecondLine = new Pose(9, 59, Math.toRadians(180));
    private final Pose controlSecondLine = new Pose(48.4, 57.4, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(56, 86, Math.toRadians(180));
    private final Pose intakeGate = new Pose(11.7, 58.2, Math.toRadians(165));
    private final Pose finishGate = new Pose(10.7, 57.4, Math.toRadians(140));
    private final Pose finishFirstLine = new Pose(14, 83.4, Math.toRadians(180));
    private Path scorePreload, grabSecond, finishSecond, scoreSecond, grabGate, finalGate, scoreGate, finishFirst, scoreFirst, park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabSecond = new Path(new BezierLine(scorePose, intakeSecondLine));
        grabSecond.setLinearHeadingInterpolation(scorePose.getHeading(), intakeSecondLine.getHeading());

        finishSecond = new Path(new BezierLine(intakeSecondLine, finishSecondLine));
        finishSecond.setConstantHeadingInterpolation(finishSecondLine.getHeading());

        scoreSecond = new Path(new BezierCurve(finishSecondLine, controlSecondLine, scorePose2));
        scoreSecond.setLinearHeadingInterpolation(finishSecondLine.getHeading() ,scorePose2.getHeading());

        grabGate = new Path(new BezierCurve(scorePose2, controlSecondLine, intakeGate));
        grabGate.setLinearHeadingInterpolation(scorePose2.getHeading(), intakeGate.getHeading());

        finalGate = new Path(new BezierLine(intakeGate, finishGate));
        finalGate.setLinearHeadingInterpolation(intakeGate.getHeading(), finishGate.getHeading());

        scoreGate = new Path(new BezierCurve(intakeGate, controlSecondLine, scorePose2));
        scoreGate.setConstantHeadingInterpolation(scorePose2.getHeading());

        finishFirst = new Path(new BezierLine(scorePose2, finishFirstLine));
        finishFirst.setConstantHeadingInterpolation(scorePose2.getHeading());

        scoreFirst = new Path(new BezierLine(finishFirstLine, scorePose2));
        scoreFirst.setConstantHeadingInterpolation(scorePose2.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.state = Shooter.State.IDLE;
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(grabSecond);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(finishSecond);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreSecond);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(grabGate);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(finalGate);
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreGate);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(grabGate);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(finalGate);
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreGate);
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(grabGate);
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(finalGate);
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreGate);
                    setPathState(17);
                }
                break;
            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(18);
                }
                break;
            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(grabGate);
                    setPathState(19);
                }
                break;
            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(finalGate);
                    setPathState(20);
                }
                break;
            case 20:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreGate);
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(22);
                }
                break;
//            case 8:
//                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
//                    follower.followPath(finishGate);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    follower.followPath(scoreGate);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    shooter.state = Shooter.State.IDLE;
//                    follower.followPath(grabGate);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(finishGate);
//                    setPathState(13);
//                }
//                break;
//            case 13:
//                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    follower.followPath(scoreGate);
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(15);
//                }
//                break;
//            case 15:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    shooter.state = Shooter.State.IDLE;
//                    follower.followPath(grabFirst);
//                    setPathState(16);
//                }
//                break;
//            case 16:
//                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    follower.followPath(scoreFirst);
//                    setPathState(17);
//                }
//                break;
//            case 17:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(18);
//                }
//                break;

        }
    }

    @Override
    public void loop() {
        shooter.update_shooter(follower.getPose().getX(), follower.getPose().getY(), gamepad1);
        turret.update_turret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        intake.update_intake(gamepad1);
        follower.update();


        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("vel ", shooter.motor_shooter.getVelocity());
        telemetry.addData(" error val ", Shooter.error);
        telemetry.update();
    }

    @Override
    public void init() {
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Globals.alliance = Globals.ALLIANCE.BLUE;
        Globals.faze = Globals.FAZE.AUTO;

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
}
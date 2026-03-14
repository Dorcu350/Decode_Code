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
public class Close_Aicit_Red extends OpMode {
    Shooter shooter;
    Intake intake;
    Sensors sensors;
    Turret turret;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(15.5, 111, Math.toRadians(180)).mirror();
    private final Pose scorePose = new Pose(47.9, 84, Math.toRadians(180)).mirror();
    private final Pose controlPose = new Pose(40.7, 105.7, Math.toRadians(180)).mirror();
    private final Pose finishFirstLine = new Pose(18, 84, Math.toRadians(180)).mirror();
    private final Pose intakeSecondLine = new Pose(47, 59, Math.toRadians(180)).mirror();
    private final Pose finishSecondLine = new Pose(9, 59, Math.toRadians(180)).mirror();
    private final Pose controlSecondLine = new Pose(48.4, 57.4, Math.toRadians(180)).mirror();
    private final Pose intakeThirdLine = new Pose(44, 35.5, Math.toRadians(180)).mirror();
    private final Pose finishThirdLine = new Pose(9, 35.5, Math.toRadians(180)).mirror();
    private final Pose openGatePose = new Pose(11, 65.6, Math.toRadians(270)).mirror();
    private final Pose controlGatePose = new Pose(21.8, 51.3, Math.toRadians(270)).mirror();
    private final Pose scorePose2 = new Pose(47.9, 84, Math.toRadians(240)).mirror();
    private final Pose firstHPPose = new Pose(20, 10.6, Math.toRadians(260)).mirror();
    private final Pose secondHPPose = new Pose(13, 9.6, Math.toRadians(260)).mirror();
    private final Pose thirdHPPose = new Pose(8, 7, Math.toRadians(270)).mirror();
    private final Pose controlThirdHP = new Pose(3.4, 66.1, Math.toRadians(270)).mirror();
    private final Pose scorePose3 = new Pose(56, 100, Math.toRadians(240)).mirror();
    private Path scorePreload, grabSecond, finishSecond, scoreSecond, finishFirst, scoreFirst, grabThird, finishThird, scoreThird, openGate, grabFirstHP, scoreFirstHP, grabSecondHP, scoreSecondHP, grabThirdHP, scoreThirdHP;

    public void buildPaths() {
        scorePreload = new Path(new BezierCurve(startPose, controlPose ,scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        finishFirst = new Path(new BezierLine(scorePose, finishFirstLine));
        finishFirst.setConstantHeadingInterpolation(scorePose.getHeading());

        scoreFirst = new Path(new BezierLine(finishFirstLine, scorePose));
        scoreFirst.setConstantHeadingInterpolation(scorePose.getHeading());

        scoreFirst = new Path(new BezierLine(finishFirstLine, scorePose));
        scoreFirst.setConstantHeadingInterpolation(scorePose.getHeading());

        grabSecond = new Path(new BezierLine(scorePose, intakeSecondLine));
        grabSecond.setLinearHeadingInterpolation(scorePose.getHeading(), intakeSecondLine.getHeading());

        finishSecond = new Path(new BezierLine(intakeSecondLine, finishSecondLine));
        finishSecond.setConstantHeadingInterpolation(finishSecondLine.getHeading());

        scoreSecond = new Path(new BezierCurve(finishSecondLine, controlSecondLine, scorePose));
        scoreSecond.setLinearHeadingInterpolation(finishSecondLine.getHeading() ,scorePose.getHeading());

        grabThird = new Path(new BezierLine(scorePose, intakeThirdLine));
        grabThird .setLinearHeadingInterpolation(scorePose.getHeading(), intakeThirdLine.getHeading());

        finishThird = new Path(new BezierLine(intakeThirdLine, finishThirdLine));
        finishThird.setConstantHeadingInterpolation(finishThirdLine.getHeading());

        scoreThird = new Path(new BezierLine(openGatePose, scorePose2));
        scoreThird.setLinearHeadingInterpolation(openGatePose.getHeading() ,scorePose2.getHeading());

        openGate = new Path(new BezierCurve(finishThirdLine, controlGatePose, openGatePose));
        openGate.setLinearHeadingInterpolation(finishThirdLine.getHeading() , openGatePose.getHeading());

        grabFirstHP = new Path(new BezierCurve(scorePose2, controlThirdHP, thirdHPPose));
//        grabFirstHP .setLinearHeadingInterpolation(scorePose2.getHeading(), thirdHPPose.getHeading());
        grabFirstHP .setTangentHeadingInterpolation();

        scoreFirstHP = new Path(new BezierLine(thirdHPPose, scorePose2));
        scoreFirstHP.setLinearHeadingInterpolation(firstHPPose.getHeading(), scorePose2.getHeading());

        grabSecondHP = new Path(new BezierLine(scorePose2, secondHPPose));
//        grabSecondHP .setLinearHeadingInterpolation(scorePose2.getHeading(), secondHPPose.getHeading());
        grabSecondHP .setTangentHeadingInterpolation();

        scoreSecondHP = new Path(new BezierLine(secondHPPose, scorePose2));
        scoreSecondHP.setLinearHeadingInterpolation(secondHPPose.getHeading(), scorePose2.getHeading());

        grabThirdHP = new Path(new BezierCurve(scorePose2, controlThirdHP,thirdHPPose));
//        grabThirdHP .setLinearHeadingInterpolation(scorePose2.getHeading(), thirdHPPose.getHeading());
        grabThirdHP .setTangentHeadingInterpolation();

        scoreThirdHP = new Path(new BezierLine(thirdHPPose, scorePose3));
        scoreThirdHP.setLinearHeadingInterpolation(thirdHPPose.getHeading(), scorePose3.getHeading());
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
                if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(finishFirst);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(scoreFirst);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 0.9) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(grabSecond);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(finishSecond);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(scoreSecond);
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 1.4) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 0.9) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(grabThird);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(finishThird);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(openGate);
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(scoreThird);
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTimeSeconds() > 1.2) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 0.7) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(grabFirstHP);
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 1.8) {
                    follower.followPath(scoreFirstHP);
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 1.6) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(17);
                }
                break;
            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 0.9) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(grabSecondHP);
                    setPathState(18);
                }
                break;
            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 2.1) {
                    follower.followPath(scoreSecondHP);
                    setPathState(19);
                }
                break;
            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 1.8) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(20);
                }
                break;
            case 20:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(grabThirdHP);
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds() > 2.2) {
                    follower.followPath(scoreThirdHP);
                    setPathState(22);
                }
                break;
            case 22:
                if(pathTimer.getElapsedTimeSeconds() > 2.2) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(23);
                }
                break;
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

        Globals.alliance = Globals.ALLIANCE.RED;
        Globals.faze = Globals.FAZE.AUTO;
        Globals.zone = Globals.ZONE.CLOSE;

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
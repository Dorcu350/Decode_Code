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

@Autonomous(group = "FAR")
public class Far_HP_Red extends OpMode {
    Shooter shooter;
    Intake intake;
    Sensors sensors;
    Turret turret;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)).mirror(); // x55.4
    private final Pose scorePose = new Pose(53.4, 13, Math.toRadians(90)).mirror();
    private final Pose scorePose180 = new Pose(56, 8, Math.toRadians(180)).mirror();
    private final Pose intakeCornerPose = new Pose(12, 8.4, Math.toRadians(180)).mirror();
    private final Pose intakeCornerPose2 = new Pose(12, 14, Math.toRadians(180)).mirror();
    private final Pose intakeLinePose = new Pose(12, 35.5, Math.toRadians(180)).mirror();
    private final Pose controlIntakePose = new Pose(47.7,35.6, Math.toRadians(180)).mirror();

    private Path scorePreload, goIntakeAfterPre, scoreCorner, goIntake, goIntake2, firstLine, scoreLine;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading() ,scorePose.getHeading());

        //===============

        goIntakeAfterPre = new Path(new BezierLine(scorePose, intakeCornerPose));
        goIntakeAfterPre.setConstantHeadingInterpolation(intakeCornerPose.getHeading());

        goIntake = new Path(new BezierLine(scorePose180, intakeCornerPose));
        goIntake.setConstantHeadingInterpolation(intakeCornerPose.getHeading());

        goIntake2 = new Path(new BezierLine(scorePose180, intakeCornerPose2));
        goIntake2.setConstantHeadingInterpolation(intakeCornerPose2.getHeading());

        scoreCorner = new Path(new BezierLine(intakeCornerPose, scorePose180));
        scoreCorner.setConstantHeadingInterpolation(scorePose180.getHeading());

        firstLine = new Path(new BezierCurve(scorePose, controlIntakePose, intakeLinePose));
        firstLine.setTangentHeadingInterpolation();

        scoreLine = new Path(new BezierCurve(intakeLinePose, controlIntakePose , scorePose180));
        scoreLine.setConstantHeadingInterpolation(scorePose180.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.state = Shooter.State.IDLE;
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 1.2) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(firstLine);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreLine);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(goIntake);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreCorner);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 0.7) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(goIntake);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreCorner);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 0.7) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(goIntake);
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreCorner);
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
                if(pathTimer.getElapsedTimeSeconds() > 0.7) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(goIntake2);
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreCorner);
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(17);
                }
                break;
            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 0.7) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(goIntake);
                    setPathState(18);
                }
                break;
            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreCorner);
                    setPathState(19);
                }
                break;
            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(20);
                }
                break;
            case 20:
                if(pathTimer.getElapsedTimeSeconds() > 0.7) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(goIntake);
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(scoreCorner);
                    setPathState(22);
                }
                break;
            case 22:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
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
        Globals.zone = Globals.ZONE.FAR;

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
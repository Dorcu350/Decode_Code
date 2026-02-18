package org.firstinspires.ftc.teamcode.StopC.Autonom;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.StopC.Subsyst.Turret;
import org.firstinspires.ftc.teamcode.StopC.Subsyst.Intake;
import org.firstinspires.ftc.teamcode.StopC.Subsyst.Sensors;
import org.firstinspires.ftc.teamcode.StopC.Subsyst.Shooter;
import org.firstinspires.ftc.teamcode.StopC.Utils.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "CLOSE")
public class AutoLeaveRed extends OpMode {
    Shooter shooter;
    Intake intake;
    Sensors sensors;
    Turret turret;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(129.4, 8.8, Math.toRadians(90));
    private final Pose scorePose = new Pose(84.2, 131.1, Math.toRadians(90));
    private final Pose intakeCornerPose = new Pose(10, 8.4, Math.toRadians(180)).mirror();
    private final Pose intakeCornerPose2 = new Pose(10, 13, Math.toRadians(200)).mirror();
    private final Pose controlIntakeCornerPose2 = new Pose(18, 16, Math.toRadians(200)).mirror();
    private final Pose stepBackPose = new Pose(17, 8.5, Math.toRadians(180)).mirror();
    private Path scorePreload, goIntake, stepBack, stepForward, scoreCorner;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading() ,scorePose.getHeading());

        goIntake = new Path(new BezierLine(scorePose, intakeCornerPose));
        goIntake.setConstantHeadingInterpolation(intakeCornerPose.getHeading());

        stepBack = new Path(new BezierLine(intakeCornerPose, stepBackPose));
        stepBack.setLinearHeadingInterpolation(intakeCornerPose.getHeading(), stepBackPose.getHeading());

        stepForward = new Path(new BezierCurve(stepBackPose, controlIntakeCornerPose2 ,intakeCornerPose2));
        stepForward.setLinearHeadingInterpolation(stepBackPose.getHeading() , intakeCornerPose2.getHeading());

        scoreCorner = new Path(new BezierLine(intakeCornerPose2, scorePose));
        scoreCorner.setConstantHeadingInterpolation(scorePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(100);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    shooter.state = Shooter.State.IDLE;
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(goIntake);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(stepBack);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(stepForward);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    intake.state = Intake.State.STOP;
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(scoreCorner);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.state = Intake.State.STOP;
                    shooter.state = Shooter.State.SHOOT;
                    setPathState(2);
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
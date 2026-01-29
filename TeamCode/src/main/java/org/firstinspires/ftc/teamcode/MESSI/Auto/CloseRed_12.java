package org.firstinspires.ftc.teamcode.MESSI.Auto;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

//@Autonomous(group = "Close")
public class CloseRed_12 extends OpMode {
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
    private final Pose scorePose = new Pose(-40, 20, Math.toRadians(-44));
    private final Pose scorePose2 = new Pose(-54, 35, Math.toRadians(-47));
    private final Pose firstPrePose = new Pose(-50, 13, 1.56);
    private final Pose firstIntake = new Pose(-48, -13, 1.56);
    private final Pose controlGate = new Pose(-55, -6, 1.56);
    private final Pose openGate = new Pose(-60, -14, 1.56);
    private final Pose secondPrePose = new Pose(-74, 10, 1.56);
    private final Pose secondIntake = new Pose(-74, -13, 1.56);
    private final Pose thirdPrePose = new Pose(-96, 10, 1.56);
    private final Pose thirdIntake = new Pose(-98, -14, 1.56);
    private final Pose parkPose = new Pose(-60, -6, 0);

    private Path scorePreload, grabFirst, finishFirst, goOpen, scoreFirst, grabSecond, finishSecond, grabThird, finishThird, scoreSecond, scoreThird, park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabFirst = new Path(new BezierCurve(scorePose, firstPrePose));
        grabFirst.setLinearHeadingInterpolation(scorePose.getHeading(), firstPrePose.getHeading());

        finishFirst = new Path(new BezierLine(firstPrePose, firstIntake));
        finishFirst.setConstantHeadingInterpolation(firstIntake.getHeading());

        goOpen = new Path(new BezierCurve(firstIntake, controlGate ,openGate));
        goOpen.setConstantHeadingInterpolation(openGate.getHeading());

        scoreFirst = new Path(new BezierCurve(openGate, scorePose ,scorePose2));
        scoreFirst.setConstantHeadingInterpolation(scorePose2.getHeading());

        grabSecond = new Path(new BezierCurve(scorePose2, secondPrePose));
        grabSecond.setLinearHeadingInterpolation(scorePose2.getHeading(), secondPrePose.getHeading());

        finishSecond = new Path(new BezierLine(secondPrePose, secondIntake));
        finishSecond.setConstantHeadingInterpolation(secondIntake.getHeading());

        grabThird = new Path(new BezierCurve(scorePose2, thirdPrePose));
        grabThird.setLinearHeadingInterpolation(scorePose2.getHeading(), thirdPrePose.getHeading());

        finishThird = new Path(new BezierLine(thirdPrePose, thirdIntake));
        finishThird.setConstantHeadingInterpolation(thirdIntake.getHeading());

        scoreSecond = new Path(new BezierCurve(secondIntake, thirdPrePose, scorePose2));
        scoreSecond.setLinearHeadingInterpolation(secondIntake.getHeading() ,scorePose2.getHeading());

        scoreThird = new Path(new BezierLine(thirdIntake, scorePose2));
        scoreThird.setConstantHeadingInterpolation(scorePose2.getHeading());

        park = new Path(new BezierLine(scorePose2, parkPose));
        park.setLinearHeadingInterpolation(scorePose2.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Shooter.auto = true;
                Shooter.check_align = false;
                follower.followPath(scorePreload);
                shooter.state = Shooter.State.IDLE;
                setPathState(1);
                break;
            case 1:
                if (littleError(scorePose.getX(), scorePose.getY()) && noHeadingError()  && pathTimer.getElapsedTimeSeconds() > 1.5)
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
                    follower.setMaxPower(0.6);
                    setPathState(3);
                }
                break;
            case 3:
                if(littleError(firstIntake.getX(), firstIntake.getY())) {
                    intake.state = Intake.State.STOP;
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(goOpen);
                    follower.setMaxPower(1);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(scoreFirst);
                    setPathState(5);
                }
                break;
            case 5:
                if (littleError(scorePose2.getX(), scorePose2.getY()) && noHeadingError())
                    shooter.state = Shooter.State.SHOOT;

                if (shooter.state == Shooter.State.SHOOT && pathTimer.getElapsedTimeSeconds() > 4) {
                    shooter.state = Shooter.State.STOPPED;
                    follower.followPath(grabSecond);
                    setPathState(6);
                }
                break;
            case 6:
                if (littleError(secondPrePose.getX(), secondPrePose.getY())) {
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(finishSecond);
                    follower.setMaxPower(0.6);
                    setPathState(7);
                }
                break;
            case 7:
                if(littleError(secondIntake.getX(), secondIntake.getY())) {
                    intake.state = Intake.State.STOP;
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(scoreSecond);
                    follower.setMaxPower(1);
                    setPathState(8);
                }
                break;
            case 8:
                if (littleError(scorePose2.getX(), scorePose2.getY()) && noHeadingError())
                    shooter.state = Shooter.State.SHOOT;

                if (shooter.state == Shooter.State.SHOOT && pathTimer.getElapsedTimeSeconds() > 4) {
                    shooter.state = Shooter.State.STOPPED;
                    follower.followPath(grabThird);
                    setPathState(9);
                }
                break;
            case 9:
                if (littleError(thirdPrePose.getX(), thirdPrePose.getY())) {
                    intake.state = Intake.State.INTAKE;
                    follower.followPath(finishThird);
                    follower.setMaxPower(0.6);
                    setPathState(10);
                }
                break;
            case 10:
                if(littleError(thirdIntake.getX(), thirdIntake.getY())) {
                    intake.state = Intake.State.STOP;
                    shooter.state = Shooter.State.IDLE;
                    follower.followPath(scoreThird);
                    follower.setMaxPower(1);
                    setPathState(11);
                }
                break;
            case 11:
                if (littleError(scorePose2.getX(), scorePose2.getY()) && noHeadingError())
                    shooter.state = Shooter.State.SHOOT;

                if (shooter.state == Shooter.State.SHOOT && pathTimer.getElapsedTimeSeconds() > 5) {
                    shooter.state = Shooter.State.STOPPED;
                    follower.followPath(park);
                    setPathState(12);
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

        sensors.setGoalRed();

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
    public boolean noHeadingError() {
        return true;
    }
    public boolean littleError(double targetX, double targetY) {
        return abs(follower.getPose().getX() - targetX) <= 1.5 && abs(follower.getPose().getY() - targetY) <= 1.5;
    }
}
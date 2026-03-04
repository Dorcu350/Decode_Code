//package org.firstinspires.ftc.teamcode.StopC.Autonom;
//
//import static java.lang.Math.abs;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.StopC.Subsyst.Turret;
//import org.firstinspires.ftc.teamcode.StopC.Subsyst.Intake;
//import org.firstinspires.ftc.teamcode.StopC.Subsyst.Sensors;
//import org.firstinspires.ftc.teamcode.StopC.Subsyst.Shooter;
//import org.firstinspires.ftc.teamcode.StopC.Utils.Globals;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(group = "Boogey")
//public class AutoBoogeyRed extends OpMode {
//    Shooter shooter;
//    Intake intake;
//    Sensors sensors;
//    Turret turret;
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//    private int pathState;
//    private final Pose startPose = new Pose(56, 8.8, Math.toRadians(90)).mirror();
//    private final Pose scorePose = new Pose(56, 12, Math.toRadians(90)).mirror();
//    private final Pose intakeLinePose = new Pose(42, 35.5, Math.toRadians(180)).mirror();
//    private final Pose finalLinePose = new Pose(11, 35.5, Math.toRadians(180)).mirror();
//    private final Pose scorePose180 = new Pose(50, 10, Math.toRadians(180)).mirror();
//    private final Pose intakeCornerPose = new Pose(10, 10, Math.toRadians(180)).mirror();
//    private final Pose intakeCornerPose2 = new Pose(10, 24, Math.toRadians(190)).mirror();
//    private final Pose controlIntakeCornerPose2 = new Pose(33.9, 28.6, Math.toRadians(190)).mirror();
//    private final Pose stepBackPose = new Pose(25, 10, Math.toRadians(180)).mirror();
//    private Path scorePreload, goLine, finishLine, scoreLine,goIntake, stepBack, stepForward, scoreCorner;
//
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading() ,scorePose.getHeading());
//
//        goLine = new Path(new BezierLine(scorePose, intakeLinePose));
//        goLine.setLinearHeadingInterpolation(scorePose.getHeading() ,intakeLinePose.getHeading());
//
//        finishLine = new Path(new BezierLine(intakeLinePose, finalLinePose));
//        finishLine.setConstantHeadingInterpolation(finalLinePose.getHeading());
//
//        scoreLine = new Path(new BezierLine(finalLinePose, scorePose180));
//        scoreLine.setLinearHeadingInterpolation(scorePose180.getHeading() ,scorePose180.getHeading());
//
//        //===============
//
//        goIntake = new Path(new BezierLine(scorePose180, intakeCornerPose));
//        goIntake.setConstantHeadingInterpolation(intakeCornerPose.getHeading());
//
//        stepBack = new Path(new BezierLine(intakeCornerPose, stepBackPose));
//        stepBack.setLinearHeadingInterpolation(intakeCornerPose.getHeading(), stepBackPose.getHeading());
//
//        stepForward = new Path(new BezierCurve(stepBackPose, controlIntakeCornerPose2, intakeCornerPose2));
//        stepForward.setLinearHeadingInterpolation(stepBackPose.getHeading() , intakeCornerPose2.getHeading());
//
//        scoreCorner = new Path(new BezierLine(intakeCornerPose2, scorePose180));
//        scoreCorner.setConstantHeadingInterpolation(scorePose180.getHeading());
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                shooter.state = Shooter.State.IDLE;
//                follower.followPath(scorePreload);
//                setPathState(1);
//                break;
//            case 1:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    shooter.state = Shooter.State.IDLE;
//                    intake.state = Intake.State.INTAKE;
//                    follower.followPath(goLine);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    follower.followPath(finishLine);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    follower.followPath(scoreLine);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
//                    intake.state = Intake.State.STOP;
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.IDLE;
//                    intake.state = Intake.State.INTAKE;
//                    follower.followPath(goIntake);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    follower.followPath(stepBack);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    follower.followPath(stepForward);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    follower.followPath(scoreCorner);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
//                    intake.state = Intake.State.STOP;
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.IDLE;
//                    intake.state = Intake.State.INTAKE;
//                    follower.followPath(goIntake);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    follower.followPath(stepBack);
//                    setPathState(13);
//                }
//                break;
//            case 13:
//                if(pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    follower.followPath(stepForward);
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    intake.state = Intake.State.STOP;
//                    follower.followPath(scoreCorner);
//                    setPathState(15);
//                }
//                break;
//            case 15:
//                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(17);
//                }
//                break;
//            case 17:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.FORCE_STOP;
//                    intake.state = Intake.State.INTAKE;
//                    follower.followPath(goIntake);
//                    setPathState(18);
//                }
//                break;
//            case 18:
//                if(pathTimer.getElapsedTimeSeconds() > 1)
//                    shooter.state = Shooter.State.STOPPED;
//                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    follower.followPath(stepBack);
//                    setPathState(19);
//                }
//                break;
//            case 19:
//                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    follower.followPath(stepForward);
//                    setPathState(20);
//                }
//                break;
////            case 20:
////                if(pathTimer.getElapsedTimeSeconds() > 1) {
////                    follower.followPath(scoreCorner);
////                    setPathState(21);
////                }
////                break;
//
//        }
//    }
//
//    @Override
//    public void loop() {
//        shooter.update_shooter(follower.getPose().getX(), follower.getPose().getY(), gamepad1);
//        turret.update_turret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
//        intake.update_intake(gamepad1);
//        follower.update();
//
//
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("vel ", shooter.motor_shooter.getVelocity());
//        telemetry.addData(" error val ", Shooter.error);
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        sensors = new Sensors(hardwareMap);
//        intake = new Intake(hardwareMap);
//        shooter = new Shooter(hardwareMap);
//        turret = new Turret(hardwareMap);
//
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        Globals.alliance = Globals.ALLIANCE.RED;
//        Globals.faze = Globals.FAZE.AUTO;
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void init_loop() {
//        telemetry.addLine("dumnezeu sa ierte victimele mele");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void stop() {
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}
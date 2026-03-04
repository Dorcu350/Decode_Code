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
//@Autonomous(group = "CLOSE")
//public class BlueCloseMax extends OpMode {
//    Shooter shooter;
//    Intake intake;
//    Sensors sensors;
//    Turret turret;
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//    private int pathState;
//    private final Pose startPose = new Pose(13, 110.7, Math.toRadians(180));
//    private final Pose scorePose = new Pose(51, 84.5, Math.toRadians(180));
//    private final Pose finishFirstLine = new Pose(16, 84, Math.toRadians(180));
//    private final Pose intakeSecondLine = new Pose(41.5, 60, Math.toRadians(180));
//    private final Pose finishSecondLine = new Pose(13, 60, Math.toRadians(180));
//    private final Pose intakeGate = new Pose(10, 61, Math.toRadians(160));
//    private final Pose strafeGate = new Pose(2.5, 54.6, Math.toRadians(130));
//    private final Pose controlGatePose = new Pose(32, 72, Math.toRadians(180));
//    private final Pose gatePose = new Pose(10, 68, Math.toRadians(180));
//    private final Pose controlSecond = new Pose(41.5, 35.7, Math.toRadians(180));
//    private final Pose scorePose2 = new Pose(58.7, 108, Math.toRadians(180));
//    private Path scorePreload, grabFirst, goOpen, scoreFirst, grabSecond, finishSecond, scoreSecond, grabGate, finishGate, scoreGate, park;
//
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        grabFirst = new Path(new BezierLine(scorePose, finishFirstLine));
//        grabFirst.setLinearHeadingInterpolation(scorePose.getHeading(), finishFirstLine.getHeading());
//
//        goOpen = new Path(new BezierCurve(finishSecondLine, controlGatePose , gatePose));
//        goOpen.setConstantHeadingInterpolation(gatePose.getHeading());
//
//        scoreFirst = new Path(new BezierLine(gatePose, scorePose2));
//        scoreFirst.setConstantHeadingInterpolation(scorePose2.getHeading());
//
//        grabSecond = new Path(new BezierLine(scorePose, intakeSecondLine));
//        grabSecond.setLinearHeadingInterpolation(scorePose.getHeading(), intakeSecondLine.getHeading());
//
//        finishSecond = new Path(new BezierLine(intakeSecondLine, finishSecondLine));
//        finishSecond.setConstantHeadingInterpolation(finishSecondLine.getHeading());
//
//        grabGate = new Path(new BezierCurve(scorePose, controlSecond, intakeGate));
//        grabGate.setLinearHeadingInterpolation(scorePose.getHeading(), intakeGate.getHeading());
//
//        finishGate = new Path(new BezierLine(intakeGate, strafeGate));
//        finishGate.setLinearHeadingInterpolation(intakeGate.getHeading() ,strafeGate.getHeading());
//
//        scoreSecond = new Path(new BezierCurve(finishSecondLine, controlSecond,scorePose));
//        scoreSecond.setLinearHeadingInterpolation(finishSecondLine.getHeading() ,scorePose.getHeading());
//
//        scoreGate = new Path(new BezierLine(strafeGate, scorePose));
//        scoreGate.setConstantHeadingInterpolation(scorePose.getHeading());
//
//        park = new Path(new BezierLine(scorePose, finishFirstLine));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), finishFirstLine.getHeading());
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
//                    follower.followPath(grabSecond);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    follower.followPath(finishSecond);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    follower.followPath(goOpen);
//                    setPathState(5);
//                }
//                //===
//            case 5:
//                if(pathTimer.getElapsedTimeSeconds() > 3) {
//                    follower.followPath(scoreSecond);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(pathTimer.getElapsedTimeSeconds() > 2) {
//                    shooter.state = Shooter.State.SHOOT;
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(pathTimer.getElapsedTimeSeconds() > 1) {
//                    shooter.state = Shooter.State.IDLE;
//                    follower.followPath(grabGate);
//                    setPathState(8);
//                }
//                break;
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
////            case 14:
////                if(pathTimer.getElapsedTimeSeconds() > 1) {
////                    shooter.state = Shooter.State.STOPPED;
////                    intake.state = Intake.State.STOP;
////                    follower.followPath(park);
////                    setPathState(15);
////                }
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
//        Globals.alliance = Globals.ALLIANCE.BLUE;
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
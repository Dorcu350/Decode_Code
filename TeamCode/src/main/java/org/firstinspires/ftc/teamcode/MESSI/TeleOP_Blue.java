package org.firstinspires.ftc.teamcode.MESSI;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Hang;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@TeleOp
@Configurable
public class TeleOP_Blue extends LinearOpMode {
    public static Pose startingPose = new Pose(0, 0, 0);
    Follower follower;
    Shooter shooter;
    Intake intake;
    Hang hang;
    Sensors sensors;
    Sorter sorter;
    ElapsedTime timer;
    Globals globals;;
    double targetHeading;
    ElapsedTime loops = new ElapsedTime();
    PIDFController controller;
    @Override
    public void runOpMode() throws InterruptedException {
        sensors = new Sensors(hardwareMap);
        hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        sorter = new Sorter(hardwareMap);
        globals = new Globals();
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();
        follower.update();

        sensors.setGoalBlue();

        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        timer.startTime();
        loops.reset();
        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while(opModeIsActive()) {
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean aPressed = gamepad1.a;

            boolean dPadUp = gamepad2.dpad_up;
            boolean dPadLeft = gamepad2.dpad_left;
            boolean leftBumper2 = gamepad2.left_bumper;
            boolean rightBumper2 = gamepad2.right_bumper;
            boolean aPressed2 = gamepad2.a;
            boolean bPressed2 = gamepad2.b;


            //FUNCTII UPDATE ----------------------------------------------------

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            shooter.update_shooter();
            intake.update_intake(gamepad1);
            hang.update();
            follower.update();

            // DRIVE ----------------------------------------------------

            double turning = (gamepad1.left_trigger - gamepad1.right_trigger) * Math.abs((gamepad1.left_trigger - gamepad1.right_trigger));

//            targetHeading = (follower.getHeading() - sensors.getTxAngle());
//            double error = targetHeading - follower.getHeading();
//            controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
//            controller.updateError(error);

            if(!globals.heading_lock) {
                Globals.heading_error = 1;

                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.1,
                        turning, //(gamepad1.left_trigger - gamepad1.right_trigger)
                        true // Robot Centric
                );
            }else {
                double error = -sensors.getTxAngle();

                controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
                controller.updateError(error);

                double turnPower = controller.run();
                Globals.heading_error = error;
                Globals.turnPowerDrive = turnPower;

                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * 0.4,
                        -gamepad1.left_stick_x * 0.4,
                        turnPower,
                        true // Robot Centric
                );
            }

            // HANG ----------------------------------------------------

            if(dPadUp && timer.milliseconds() > 250) {
                if(hang.state == Hang.State.DEFAULT)
                    hang.state = Hang.State.ENGAGED;
                else if(hang.state == Hang.State.ENGAGED)
                    hang.state = Hang.State.DEFAULT;

                timer.reset();
            }

            if(dPadLeft && timer.milliseconds() > 250) {
                if(hang.state == Hang.State.DEFAULT)
                    hang.state = Hang.State.DEFENSE;
                else if(hang.state == Hang.State.DEFENSE)
                    hang.state = Hang.State.DEFAULT;

                timer.reset();
            }

            // FAILSAFES ----------------------------------------------------

            if(leftBumper2 && timer.milliseconds() > 250) {
                intake.state = Intake.State.REVERSE;
                timer.reset();
            }

            if(rightBumper2 && timer.milliseconds() > 250) {
                shooter.goDownKickerAAA();
                timer.reset();
            }

            if(aPressed2)
                shooter.shooterSpeedFailsafe(+20);

            if(bPressed2)
                shooter.shooterSpeedFailsafe(-20);

            // SHOOTER ----------------------------------------------------

            if(rightBumper && timer.milliseconds() > 250) {
                if(shooter.state == Shooter.State.STOPPED) {
                    Shooter.initial_release = true;
                    shooter.state = Shooter.State.IDLE;
                }
                else if(shooter.state == Shooter.State.IDLE) {
                    shooter.state = Shooter.State.SHOOT;
                    globals.heading_lock = !globals.heading_lock;
                }
                else {
                    shooter.state = Shooter.State.STOPPED;
                    globals.heading_lock = !globals.heading_lock;
                }

                timer.reset();
            }

            if(gamepad1.b)
                shooter.state = Shooter.State.STOPPED;

            // INTAKE  ----------------------------------------------------

//            if(gamepad1.dpad_left && timer.milliseconds() > 250) {
//                globals.sorter_active = !globals.sorter_active;
//                timer.reset();
//            }

            if(leftBumper && !shooter.is_spinning()) {
                if (intake.is_intaking() && timer.milliseconds() > 250) {
                    intake.state = Intake.State.STOP;
                    timer.reset();
                } else if (!intake.is_intaking() && !globals.sorter_active && timer.milliseconds() > 250) {
                    intake.state = Intake.State.INTAKE;
                    timer.reset();
                } else if (!intake.is_intaking() && globals.sorter_active && timer.milliseconds() > 250) {
                    intake.state = Intake.State.INTAKE_WHILE_SORTING;
                    sorter.openLatch();
                    timer.reset();
                }
            }

            if(aPressed)
                intake.daPush = !intake.daPush;


            // TELEMETRY ----------------------------------------------------

//            sorter.closeLatch();

//            telemetry.addData("flywheel state running ", shooter.state == Shooter.State.STOPPED);
//            telemetry.addData("flywheel state shoot ", shooter.state == Shooter.State.SHOOT);
//            telemetry.addData("flywheel state idle ", shooter.state == Shooter.State.IDLE);
            telemetry.addData("sensor feed ", sensors.check_for_shooting());
            telemetry.addData("vel ", shooter.motor_shooter.getVelocity());
//            telemetry.addData("kicker pos ", sensors.readKickerPos()); //0.0006
//            telemetry.addData("pozitie servo ", shooter.servo_feeder.getPosition());
//            telemetry.addData("rpm ", shooter.getRpm(shooter.motor_shooter.getVelocity()));
            telemetry.addData("tA ", sensors.getTa());
//            telemetry.addData("timer ", intake.timer.milliseconds());
//            telemetry.addData("tx ", sensors.getTx());
            telemetry.addData("is kicker down ", sensors.kickerRetracted());
//            telemetry.addData("error ", shooter.noError(Shooter.error));
            telemetry.addData("target vel ", Shooter.target_velocity);
//            telemetry.addData("heading ", follower.getHeading());
            telemetry.addData(" error val ", Shooter.error);
//            telemetry.addData("hang poz ", hang.motor_hang.getCurrentPosition());xx
//            telemetry.addData("current ", shooter.motor_shooter.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("current intake ", intake.motor_intake.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("loop times ", loops.milliseconds());
            telemetry.addData("zone ", shooter.zone.getClosest(Shooter.lastValue));
            telemetry.addData("globals heading error ", Globals.heading_error);
            telemetry.addData("has shot ", Shooter.has_shot);
            telemetry.addData("turn power drive ", Globals.turnPowerDrive);
            telemetry.addData("intake pow ", intake.motor_intake.getPower());

//            telemetry.addData("eroare heading ", error);
            loops.reset();
//            telemetry.addData("in sorting ", sensors.check_in_sorting());
            //            telemetry.addData("heading ", follower.getHeading());
//            telemetry.addData("tx ", sensors.getTx());
//            telemetry.addData("sug pula lok", globals.heading_lock);
//            telemetry.addData("red ", sensors.showRed());
//            telemetry.addData("blue " ,sensors.showBlue());
//            telemetry.addData("green ", sensors.showGreen());
//            telemetry.addData("este verde ", sensors.isGreen());
//            telemetry.addData("este verde ", sensors.isPurple());
//            telemetry.addData("hue ", sensors.showHSV());
//            telemetry.addData("sorter activ ", globals.sorter_active);
//            telemetry.addData("is green ", sensors.isGreen());
//            telemetry.addData("is purple ", sensors.isPurple());
//            telemetry.addData("true tA, ", Shooter.lastValue);
//            telemetry.addData("first ", intake.firstBall);
//            telemetry.addData("second ", intake.secondBall);
//            telemetry.addData("hue ", sensors.showHue());
            telemetry.update();
        }
    }
}


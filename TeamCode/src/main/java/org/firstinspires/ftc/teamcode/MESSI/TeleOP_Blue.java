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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;
import org.firstinspires.ftc.teamcode.MESSI.pedroPathing.Constants;

@TeleOp
@Configurable
public class TeleOP_Blue extends LinearOpMode {
    public static Pose startingPose = new Pose(0, 0, 0);
    Follower follower;
    Shooter shooter;
    Intake intake;
    Sensors sensors;
    Sorter sorter;
    ElapsedTime timer, timer_kick;
    Globals globals;;
    double targetHeading;
    PIDFController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        sorter = new Sorter(hardwareMap);
        globals = new Globals();
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer_kick = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();
        follower.update();

        sensors.setGoalBlue();

        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        timer.startTime();
        timer_kick.startTime();

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while(opModeIsActive()) {
            //FUNCTII UPDATE ----------------------------------------------------

            shooter.update_shooter();
            intake.update_intake(gamepad1);
            follower.update();

            // DRIVE ----------------------------------------------------

            targetHeading = (follower.getHeading() - sensors.getTx());
            double error = targetHeading - follower.getHeading();
            double turning = (gamepad1.left_trigger - gamepad1.right_trigger) * Math.abs((gamepad1.left_trigger - gamepad1.right_trigger));
            controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            controller.updateError(error);

            if(!globals.heading_lock) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.1,
                        turning, //(gamepad1.left_trigger - gamepad1.right_trigger)
                        true // Robot Centric
                );
            }else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * 0.4,
                        -gamepad1.left_stick_x * 0.4,
                        controller.run(),
                        true // Robot Centric
                );
            }

            // SHOOTER ----------------------------------------------------

            if(gamepad1.right_bumper && timer.milliseconds() > 250) {
                if(!shooter.is_spinning()) {
                    shooter.state = Shooter.State.SHOOT;
                }
                else {
                    shooter.state = Shooter.State.STOPPED;
                }
                globals.heading_lock = !globals.heading_lock;
                timer.reset();
            }

            // INTAKE ----------------------------------------------------

            if(gamepad1.dpad_left && timer.milliseconds() > 250) {
                globals.sorter_active = !globals.sorter_active;
                timer.reset();
            }

            if(gamepad1.left_bumper) {
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

            if(gamepad1.a && timer.milliseconds() > 250) {
                intake.daPush = !intake.daPush;
                timer.reset();
            }

            // TELEMETRY ----------------------------------------------------

//            sorter.closeLatch();

//            telemetry.addData("flywheel state running ", shooter.state == Shooter.State.RUNNING);
//            telemetry.addData("flywheel state shoot ", shooter.state == Shooter.State.SHOOT);
//            telemetry.addData("flywheel state idle ", shooter.state == Shooter.State.IDLE);
            telemetry.addData("sensor feed ", sensors.check_for_shooting());
//            telemetry.addData("vel ", shooter.motor_shooter.getVelocity());
            telemetry.addData("tA ", sensors.getTa());
            telemetry.addData("timer ", intake.timer.milliseconds());
            telemetry.addData("current ", shooter.motor_shooter.getCurrent(CurrentUnit.MILLIAMPS));
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


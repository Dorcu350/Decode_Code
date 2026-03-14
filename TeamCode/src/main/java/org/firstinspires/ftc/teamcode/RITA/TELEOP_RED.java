package org.firstinspires.ftc.teamcode.RITA;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.RITA.Utils.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class TELEOP_RED extends LinearOpMode {
    public static Pose startingPose = new Pose(9, 8.5, Math.toRadians(0));

    public static Follower follower;
    Shooter shooter;
    Intake intake;
    Sensors sensors;
    Turret turret;
    ElapsedTime timer;
    ElapsedTime loops = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(sensors.pinpoint.getPosX(DistanceUnit.INCH), sensors.pinpoint.getPosY(DistanceUnit.INCH), sensors.pinpoint.getHeading(AngleUnit.RADIANS)));
        follower.startTeleopDrive();
        follower.update();

        Globals.alliance = Globals.ALLIANCE.RED;
        Globals.faze = Globals.FAZE.TELEOP;

        timer.startTime();
        loops.reset();
//        telemetry.setMsTransmissionInterval(11);

        telemetry.addLine("Te aves bahtalo, patroane!");
        telemetry.update();

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while(opModeIsActive()) {
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean dPadLeft = gamepad1.dpad_left;
            boolean bPressed = gamepad1.b;
            boolean xPressed = gamepad1.x;

            boolean dPadUp = gamepad2.dpad_up;
            boolean leftBumper2 = gamepad2.left_bumper;
            boolean rightBumper2 = gamepad2.right_bumper;
            boolean aPressed2 = gamepad2.a;
            boolean yPressed2 = gamepad2.y;
            boolean xPressed2 = gamepad2.x;


            //FUNCTII UPDATE ----------------------------------------------------

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            shooter.update_shooter(follower.getPose().getX(), follower.getPose().getY(), gamepad1);
            turret.update_turret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            intake.update_intake(gamepad1);

            follower.update();

            // DRIVE ----------------------------------------------------

            double turning = (gamepad1.left_trigger - gamepad1.right_trigger) * Math.abs((gamepad1.left_trigger - gamepad1.right_trigger));

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x * 1.1,
                    turning, // (gamepad1.left_trigger - gamepad1.right_trigger)
                    true // Robot Centric
            );


            // FAILSAFES ----------------------------------------------------

            if(dPadLeft && timer.milliseconds() > 250)
                follower.setPose(startingPose);

            if(rightBumper2 && timer.milliseconds() > 250) {
                Turret.offset += 0.001;
                timer.reset();
            }

            if(leftBumper2 && timer.milliseconds() > 250) {
                Turret.offset -= 0.001;
                timer.reset();
            }

            if(aPressed2)
                intake.state = Intake.State.FORCE_REVERSE;

            if(xPressed2 && timer.milliseconds() > 250) {
                Shooter.offset -= 20;
                timer.reset();
            }

            if(yPressed2 && timer.milliseconds() > 250) {
                Shooter.offset += 20;
                timer.reset();
            }

            // SHOOTER ----------------------------------------------------

            if(rightBumper && timer.milliseconds() > 250) {
                if(shooter.state == Shooter.State.IDLE)
                    shooter.state = Shooter.State.SHOOT;
                else if(shooter.state == Shooter.State.SHOOT)
                    shooter.state = Shooter.State.IDLE;

                timer.reset();
            }

            if (bPressed) {
                shooter.state = Shooter.State.IDLE;
            }

            // INTAKE  ----------------------------------------------------

            if(leftBumper && !shooter.is_spinning() && timer.milliseconds() > 250) {
                if (intake.is_intaking())
                    intake.state = Intake.State.STOP;
                else
                    intake.state = Intake.State.INTAKE;

                timer.reset();
            }

            if(xPressed && timer.milliseconds() > 250) {
                if (intake.is_intaking())
                    intake.state = Intake.State.STOP;
                else
                    intake.state = Intake.State.FORCE_REVERSE;

                timer.reset();
            }

            if(dPadUp && timer.milliseconds() > 250) {
                Globals.hanging = true;
                intake.state = Intake.State.HANG;
                shooter.state = Shooter.State.FORCE_STOP;
            }

            Globals.force_drop = gamepad1.a;

            // TELEMETRY ----------------------------------------------------

//            telemetry.addData("flywheel state running ", shooter.state == Shooter.State.STOPPED);
//            telemetry.addData("flywheel state shoot ", shooter.state == Shooter.State.SHOOT);
//            telemetry.addData("flywheel state idle ", shooter.state == Shooter.State.IDLE);
//            telemetry.addData("sensor feed ", sensors.check_for_shooting());
            telemetry.addData("vel ", shooter.motor_shooter.getVelocity());
            telemetry.addData(" error val ", Shooter.error);
//            telemetry.addData("tur off ", Turret.offset);
//            telemetry.addData("kicker pos ", sensors.readKickerPos()); //0.0006
//            telemetry.addData("pozitie servo ", shooter.servo_feeder.getPosition());
//            telemetry.addData("rpm ", shooter.getRpm(shooter.motor_shooter.getVelocity()));
//            telemetry.addData("tA ", sensors.getTa());
//            telemetry.addData("timer ", intake.timer.milliseconds());
//            telemetry.addData("tx ", sensors.getTx());
//            telemetry.addData("is kicker down ", sensors.kickerRetracted());
//            telemetry.addData("error ", shooter.noError(Shooter.error));
            telemetry.addData("target vel ", Shooter.TARGET_VELOCITY);
//            telemetry.addData("heading ", follower.getHeading());
//            telemetry.addData(" error val ", Shooter.error);
//            telemetry.addData("hang poz ", hang.motor_hang.getCurrentPosition());xx
//            telemetry.addData("current ", shooter.motor_shooter.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("current intake ", intake.motor_intake.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("loop times ", loops.milliseconds());
            telemetry.addData(" error val ", Shooter.error);
//            telemetry.addData("zone ", shooter.zone.getClosest(Shooter.lastValue));
//            telemetry.addData("globals heading error ", Globals.heading_error);
//            telemetry.addData("has shot ", Shooter.has_shot);
//            telemetry.addData("turn power drive ", Globals.turnPowerDrive);
//            telemetry.addData("intake pow ", intake.motor_intake.getPower());
//            telemetry.addData("eroare heading ", error);
            loops.reset();
//            telemetry.addData("theta ", Turret.target_angle);
//            telemetry.addData("theta degrees ", Math.toDegrees(Turret.target_angle));
//            telemetry.addData("theta - heading ", Turret.target_angle - follower.getHeading());
//            telemetry.addData("heading ", follower.getHeading());
//            telemetry.addData("heading deg ", Math.toDegrees(follower.getHeading()));
//            telemetry.addData("theta - heading degrees", Math.toDegrees(Turret.target_angle - follower.getHeading()));
//            telemetry.addData("is running intake: ", intake.state == Intake.State.INTAKE);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("relative angle normalized ", Turret.relative_angle);
            telemetry.addData("stopper open ", sensors.stopperOpen());
            telemetry.addData("max rpm ", Shooter.max_rpm);
            telemetry.addData("distance from goal ", shooter.distance_from_goal);
            telemetry.addData("target ", Shooter.TARGET_TEST);
            telemetry.addData("pow ", shooter.motor_shooter.getPower());
//            telemetry.addData("error ", Shooter.error);
//            telemetry.addData("ready to shoot ", sensors.checkForShooting());
//            telemetry.addData("desired ", Shooter.desired);
//            telemetry.addData("target pos ", Turret.target_position);
//            telemetry.addData("turret a ", sensors.readTurretAnalog());
//            telemetry.addData("globals x", Globals.curr_x);
//            telemetry.addData("globals y ", Globals.curr_y);
//            telemetry.addData("globals heading", Globals.curr_heading);
//            telemetry.addData("increment ", Turret.increment2);
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
            telemetry.addData("vol shooter" , shooter.motor_shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("transfer speed ", Intake.RATIO_SCALE);
            telemetry.update();
        }
    }
}


//package org.firstinspires.ftc.teamcode.StopC.Subsyst;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.StopC.Utils.Globals;
//
//import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
//import dev.frozenmilk.dairy.cachinghardware.CachingServo;
//
//@Configurable
//public class Intake {
//    final TelemetryManager telemetryM;
//    public CachingDcMotorEx motor_intake;
//    public CachingServo servo_shifter;
//    public static double shifter_intake = 0.07, shifter_hang = 0.13, kP = 0.1, kF = 0.1;
//    public static int target_hang = 800;
//    VoltageSensor voltageSensor;
//    Sensors sensors;
//    public enum State {
//        INTAKE,
//        STOP,
//        FORCE_REVERSE,
//        HANG
//    }
//    public State state;
//    public void update_intake(Gamepad gamepad) {
//        switch(state) {
//            case STOP:
//                shift_intake();
//                motor_intake.setPower(0);
//
//                if(Globals.start_feeding)
//                    state = State.INTAKE;
//                break;
//            case INTAKE:
//                shift_intake();
//                motor_intake.setPower(1);
//
//                if(is_over_current())
//                    gamepad.rumble(250);
//                break;
//            case FORCE_REVERSE:
//                motor_intake.setPower(-1);
//                break;
//            case HANG:
//                shift_hang();
//                if(sensors.shifterInHang()) {
//                    motor_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    motor_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////                    motor_intake.setTargetPosition(0);
////                    motor_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////                    motor_intake.setTargetPosition(target_hang);
////                    motor_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                    motor_intake.setPower(1);
//                    double error = target_hang - motor_intake.getCurrentPosition();
//                    motor_intake.setPower((kP * error ) + kF);
//                }
//                else
//                    motor_intake.setPower(0.5);
//                break;
//        }
//    }
//    public void shift_hang() {
//        servo_shifter.setPosition(shifter_hang);
//    }
//    public void shift_intake() {
//        servo_shifter.setPosition(shifter_intake);
//    }
//    public boolean is_intaking() {
//        return motor_intake.getPower() != 0;
//    }
//    public double get_current() {return motor_intake.getCurrent(CurrentUnit.MILLIAMPS);}
//    public boolean is_over_current() {return get_current() > 2500;}
//    public Intake(HardwareMap hardwareMap) {
//        motor_intake  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"));
//        servo_shifter = new CachingServo(hardwareMap.get(Servo.class, "shifter"));
//
//        motor_intake.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.BRAKE);
//        motor_intake.setDirection            (DcMotorSimple.Direction.REVERSE);
//        motor_intake.setCachingTolerance     (0.05);
//
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        shift_intake();
//
//        state = State.STOP;
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        sensors = new Sensors(hardwareMap);
//    }
//}

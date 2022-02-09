package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="SAutoPathBL", group="Linear Opmode")

public class SAutoPathBL extends LinearOpMode {
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
    private Servo gate,bucketAngle;
    private TouchSensor hardstop;

    private final double TICKS_PER_REV = 537.6;
    private final double MM_PER_REV = Math.PI * 96;
    private final double MM_PER_TILE = 609.6;

    @Override
    public void runOpMode() {
        tlMotor     = hardwareMap.get(DcMotor.class,  "tl_motor");
        trMotor     = hardwareMap.get(DcMotor.class,  "tr_motor");
        blMotor     = hardwareMap.get(DcMotor.class,  "bl_motor");
        brMotor     = hardwareMap.get(DcMotor.class,  "br_motor");
        intake      = hardwareMap.get(DcMotor.class,  "intake");
        corner      = hardwareMap.get(DcMotor.class,  "corner");
        slide       = hardwareMap.get(DcMotor.class,  "slide");
        gate        = hardwareMap.get(Servo.class,    "gate");
        bucketAngle = hardwareMap.get(Servo.class,    "bucketAngle");
        hardstop    = hardwareMap.get(TouchSensor.class, "hardstop");

        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setDirection(DcMotor.Direction.FORWARD);
        corner.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gate.setDirection(Servo.Direction.FORWARD);
        bucketAngle.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        strafe(1.0);
        drive(0.75);
        turnNinety(1);

        /*try {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while ( hardstop.isPressed() ) {
                slide.setPower(-0.5);
            }
            slide.setPower(0);
            slide.setTargetPosition((int) (-6 * TICKS_PER_REV));
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            while ( slide.isBusy() ) {}
            bucketAngle.setPosition(0.0);
            TimeUnit.MILLISECONDS.sleep(250);
            gate.setPosition(0.75);
            TimeUnit.MILLISECONDS.sleep(1000);
            bucketAngle.setPosition(0.15);
            TimeUnit.MILLISECONDS.sleep(250);
            gate.setPosition(0.0);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } catch ( InterruptedException e ) {}*/

        /*slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while ( ! hardstop.isPressed() ) {
            slide.setPower(0.5);
        }
        slide.setPower(0);*/

        drive(2.3,0.75);
        strafe(0.85,0.2);

        corner.setPower(-1.0);
        try {
            TimeUnit.MILLISECONDS.sleep(3000);
        } catch ( InterruptedException e ) {}
        corner.setPower(0.0);

        drive(-1.0);
        strafe(0.6);
        //strafe(-0.05); // To compensate for edges of mat

        drive(-3.2);
    }

    private void drive(double distance) { drive(distance,1); }
    private void strafe(double distance) { strafe(distance,1); }

    private void drive(double distance,double speed) {
        moveMotors(distance,distance,distance,distance,speed);
    }

    private void strafe(double distance,double speed) { // left is negative, right is positive
        moveMotors(distance,-distance,-distance,distance,speed);
    }

    private void turnNinety(int turns) {
        moveMotors(0.825 * turns,-0.825 * turns,0.825 * turns,-0.825 * turns,1);
    }

    private void moveMotors(double tlMove,double trMove,double blMove,double brMove,double speed) {
        tlMotor.setTargetPosition((int) (-tlMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));
        trMotor.setTargetPosition((int) (trMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));
        blMotor.setTargetPosition((int) (-blMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));
        brMotor.setTargetPosition((int) (brMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));
        tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tlMotor.setPower(speed);
        trMotor.setPower(speed);
        blMotor.setPower(speed);
        brMotor.setPower(speed);
        while ( tlMotor.isBusy() || trMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy() ) {}
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

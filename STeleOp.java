package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="STeleOp", group="Iterative Opmode")
public class STeleOp extends OpMode {
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
    private Servo gate,bucketAngle;

    @Override
    public void init() {
        tlMotor     = hardwareMap.get(DcMotor.class,  "tl_motor");
        trMotor     = hardwareMap.get(DcMotor.class,  "tr_motor");
        blMotor     = hardwareMap.get(DcMotor.class,  "bl_motor");
        brMotor     = hardwareMap.get(DcMotor.class,  "br_motor");
        intake      = hardwareMap.get(DcMotor.class,  "intake");
        corner      = hardwareMap.get(DcMotor.class,  "corner");
        slide       = hardwareMap.get(DcMotor.class,  "slide");
        gate        = hardwareMap.get(Servo.class,    "gate");
        bucketAngle = hardwareMap.get(Servo.class,    "bucketAngle");

        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        trMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        corner.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        gate.setDirection(Servo.Direction.FORWARD);
        bucketAngle.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        intake.setPower(-1.0);
    }

    @Override
    public void loop() {
        double strafe = gamepad1.left_stick_x;
        if ( Math.abs(strafe) > 0.5 ) {
            tlMotor.setPower(-strafe);
            trMotor.setPower(-strafe);
            blMotor.setPower(strafe);
            brMotor.setPower(strafe);
        } else {
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            tlMotor.setPower(drive + turn);
            trMotor.setPower(drive - turn);
            blMotor.setPower(drive + turn);
            brMotor.setPower(drive - turn);
        }

        if ( gamepad1.left_trigger > 0.0 ) slide.setPower(-0.5 * gamepad1.left_trigger);
        else if ( gamepad1.right_trigger > 0.0 ) slide.setPower(0.5 * gamepad1.right_trigger);
        else slide.setPower(0.0);

        if ( gamepad1.dpad_left ) corner.setPower(1.0);
        else if ( gamepad1.dpad_right ) corner.setPower(-1.0);
        else corner.setPower(0.0);

        if ( gamepad1.b ) {
            gate.setPosition(0.75);
            bucketAngle.setPosition(0.0);
        } else {
            gate.setPosition(0.0);
            bucketAngle.setPosition(0.175);
        }
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

@TeleOp(name="STeleOp", group="Iterative Opmode")
public class STeleOp extends OpMode {
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
    private Servo gate,bucketAngle;
    private TouchSensor hardstop;
    private ElapsedTime runtime = new ElapsedTime();
    private int dumpState = -1;
    private double dumpTime;

    private final double TICKS_PER_REV = 537.6;

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
        hardstop    = hardwareMap.get(TouchSensor.class,"hardstop");

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
        bucketAngle.setPosition(0.0);
    }

    @Override
    public void loop() {
        double strafe = gamepad1.left_stick_x;
        double drive = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x * 0.5;
        tlMotor.setPower(drive + strafe - turn);
        trMotor.setPower(drive - strafe + turn);
        blMotor.setPower(drive - strafe - turn);
        brMotor.setPower(drive + strafe + turn);

        if ( dumpState == -1 ) {
          bucketAngle.setPosition(0.0);
          if ( gamepad1.right_trigger > 0.0 ) slide.setPower(-1.0 * gamepad1.right_trigger);
          else if ( gamepad1.left_trigger > 0.0 && ! hardstop.isPressed() ) slide.setPower(0.5 * gamepad1.left_trigger);
          else slide.setPower(0.0);
        }

        if ( gamepad1.dpad_left ) corner.setPower(1.0);
        else if ( gamepad1.dpad_right ) corner.setPower(-1.0);
        else corner.setPower(0.0);

        telemetry.addData("dumpState",dumpState);
        telemetry.update();

        if ( dumpState == -1 && gamepad1.b ) {
          dumpState = 0;
          slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          slide.setTargetPosition((int) (-4.0 * TICKS_PER_REV));
          slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          slide.setPower(1.0);
        }
        if ( dumpState == 0 && ! slide.isBusy() ) {
          dumpState = 1;
          dumpTime = runtime.seconds();
          bucketAngle.setPosition(1.0);
        }
        if ( dumpState == 1 && runtime.seconds() - dumpTime >= 1.0 ) {
          dumpState = 2;
          dumpTime = runtime.seconds();
          bucketAngle.setPosition(0.0);
        }
        if ( dumpState == 2 && runtime.seconds() - dumpTime >= 0.5 ) {
          dumpState = 3;
          slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          slide.setTargetPosition((int) (4.0 * TICKS_PER_REV));
          slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          slide.setPower(1.0);
        }
        if ( dumpState == 3 && ! slide.isBusy() ) {
          dumpState = 4;
          slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          slide.setPower(0.125);
        }
        if ( dumpState == 4 && hardstop.isPressed() ) {
          dumpState = -1;
          slide.setPower(0.0);
        }

        if ( ! gamepad1.right_bumper ) intake.setPower(1.0);
        else intake.setPower(-1.0);
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
    }
}

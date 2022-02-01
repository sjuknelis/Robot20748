package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Basic: Iterative OpMode", group="Iterative Opmode")

public class SAutoPath extends OpMode {
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
    private Servo gate,bucketAngle;
    private int tlMotorPosition = 0;
    private int trMotorPosition = 0;
    private int blMotorPosition = 0;
    private int brMotorPosition = 0;
    
    private final double TICKS_PER_REV = 537.6;
    private final double MM_PER_REV = Math.PI * 96;
    
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
        
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        int distance = (int) ((1219.2 / MM_PER_REV) * TICKS_PER_REV);
        
        
        
        intake.setPower(-1.0);
    }

    private void moveMotors(int tlMove,int trMove,int blMove,int brMove) {
        tlMotor.setTargetPosition(tlMotorPosition + tlMove);
        trMotor.setTargetPosition(trMotorPosition + trMove);
        blMotor.setTargetPosition(blMotorPosition + blMove);
        brMotor.setTargetPosition(brMotorPosition + brMove);
        tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tlMotor.setPower(1);
        trMotor.setPower(1);
        blMotor.setPower(1);
        brMotor.setPower(1);
        while ( tlMotor.isBusy() || trMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy() ) {}
    }

    @Override
    public void loop() {
        
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
    }
}

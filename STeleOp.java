package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="STeleOp", group="Iterative Opmode")

public class STeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        tlMotor = hardwareMap.get(DcMotor.class, "tl_motor");
        trMotor = hardwareMap.get(DcMotor.class, "tr_motor");
        blMotor = hardwareMap.get(DcMotor.class, "bl_motor");
        brMotor = hardwareMap.get(DcMotor.class, "br_motor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        corner = hardwareMap.get(DcMotor.class, "corner");
        slide = hardwareMap.get(DcMotor.class, "slide");
        
        tlMotor.setDirection(DcMotor.Direction.FORWARD);
        trMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        corner.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        intake.setPower(-1.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double strafe = gamepad1.left_stick_x;
        if ( strafe != 0. ) {
            tlMotor.setPower(-strafe);
            trMotor.setPower(-strafe);
            blMotor.setPower(strafe);
            brMotor.setPower(strafe);
        } else {
            double drive = gamepad1.right_stick_y;
            double turn = gamepad1.right_stick_x;
            tlMotor.setPower(drive + turn);
            trMotor.setPower(drive - turn);
            blMotor.setPower(drive + turn);
            brMotor.setPower(drive - turn);
        }
        
        if ( gamepad1.left_bumper ) slide.setPower(0.5);
        else if ( gamepad1.right_bumper ) slide.setPower(-0.5);
        else slide.setPower(0.0);
        
        if ( gamepad1.dpad_left ) corner.setPower(1.0);
        else if ( gamepad1.dpad_right ) corner.setPower(-1.0);
        else corner.setPower(0.0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        intake.setPower(0.0);
    }

}

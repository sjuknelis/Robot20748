package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="STeleOp", group="Iterative Opmode")
public class STeleOp extends OpMode {
  // Variables that represent the motors, servos, sensors
  private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide,secondIntake;
  private Servo gate,bucketAngle;
  private TouchSensor hardstop;

  // General variables to be used later
  private ElapsedTime runtime = new ElapsedTime();
  private double startTime = 0;
  private int dumpState = -1;
  private double dumpTime;

  // The encoders in our motors tick 537.6 times per revolution
  // Our wheels are 96mm in diameter
  // A tile is 24 inches
  private final double TICKS_PER_REV = 537.6;
  private final double MM_PER_REV = Math.PI * 96;
  private final double MM_PER_TILE = 609.6;

  @Override
  public void init() {
    // Load the motors, servos, sensor into variables
    tlMotor       = hardwareMap.get(DcMotor.class,      "tl_motor");
    trMotor       = hardwareMap.get(DcMotor.class,      "tr_motor");
    blMotor       = hardwareMap.get(DcMotor.class,      "bl_motor");
    brMotor       = hardwareMap.get(DcMotor.class,      "br_motor");
    intake        = hardwareMap.get(DcMotor.class,      "intake");
    corner        = hardwareMap.get(DcMotor.class,      "corner");
    slide         = hardwareMap.get(DcMotor.class,      "slide");
    secondIntake  = hardwareMap.get(DcMotor.class,      "secondIntake");
    gate          = hardwareMap.get(Servo.class,        "gate");
    bucketAngle   = hardwareMap.get(Servo.class,        "bucketAngle");
    hardstop      = hardwareMap.get(TouchSensor.class,  "hardstop");

    // The way the drivetrain motors are wired up, the ones on the left go backwards
    // (We reversed - and + on the left because it's just easier to wire like that)
    tlMotor.setDirection(DcMotor.Direction.REVERSE);
    trMotor.setDirection(DcMotor.Direction.FORWARD);
    blMotor.setDirection(DcMotor.Direction.REVERSE);
    brMotor.setDirection(DcMotor.Direction.FORWARD);

    // All the other motors are just forward
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
    // For the bucket, 0 is the bottom and 1 is all the way over the top (dumping)
    bucketAngle.setPosition(0.0);
  }

  @Override
  public void loop() {
    // gamepad1.left_stick_x etc. are variables for joystick positions represented in the range [-1,1]
    double strafe = gamepad1.left_stick_x;
    double drive = -gamepad1.left_stick_y;
    double turn = -gamepad1.right_stick_x * 0.5;
    // Driving is simple, you just go forward or backward on all motors
    // If you wanna understand the strafing, look at https://files.andymark.com/PDFs/MecanumWheelTutorial.pdf, but you don't really need to, I got the pluses and minuses by guessing
    // To turn in place to the left, you put left wheels backward and right wheels forward
    tlMotor.setPower(drive + strafe - turn);
    trMotor.setPower(drive - strafe + turn);
    blMotor.setPower(drive - strafe - turn);
    brMotor.setPower(drive + strafe + turn);

    if ( dumpState == -1 ) {  // If we're not currently dumping
      bucketAngle.setPosition(0.0); // Forcibly keep the bucket in the bottom position, even if pushed by a box
      // Manual control over the slide going up on L2 and down on R2
      if ( gamepad1.right_trigger > 0.0 ) slide.setPower(-1.0 * gamepad1.right_trigger);
      else if ( gamepad1.left_trigger > 0.0 && ! hardstop.isPressed() ) slide.setPower(0.5 * gamepad1.left_trigger); // Don't go further back if the slide is already at the bottom and touching the hardstop button
      else slide.setPower(0.0);
    }

    // Spin the duck carousel with the corner motor
    if ( gamepad1.dpad_left ) corner.setPower(1.0);
    else if ( gamepad1.dpad_right ) corner.setPower(-1.0);
    else corner.setPower(0.0);

    // The dumpState variable works by going -1 -> 0 -> 1 -> 2 -> 3 -> 4 -> -1, where -1 means we're not currently dumping
    // My original code was written by just saying move the slide, then wait until the slide finished moving, then move the bucket...
    // The problem with that was when you were dumping, the code was just waiting in an empty loop, so you couldn't adjust your position while dumping
    // Using this code, all the normal movement code still runs during the dumping process
    if ( dumpState == -1 && gamepad1.b ) {
      dumpState = 0;
      // Put the encoder back to zero
      // The encoder is always tracking whenever the motor moves, so if you want to move the motor 3.5 revolutions,
      // you have to set the encoder to zero and then tell it to go to 3.5 revolutions
      slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slide.setTargetPosition((int) (-3.5 * TICKS_PER_REV));
      slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Run the motor to the target position
      slide.setPower(1.0);
    }
    if ( dumpState == 0 && ! slide.isBusy() ) {
      dumpState = 1;
      dumpTime = runtime.seconds(); // runtime.seconds() gets the seconds since the beginning of the match
      bucketAngle.setPosition(1.0);
    }
    if ( dumpState == 1 && runtime.seconds() - dumpTime >= 1.0 ) {  // If 1 second has passed
      dumpState = 2;
      dumpTime = runtime.seconds();
      bucketAngle.setPosition(0.0);
    }
    if ( dumpState == 2 && runtime.seconds() - dumpTime >= 0.5 ) {  // If 0.5 seconds have passed
      dumpState = 3;
      slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // -3.5 is to lift the slide up, so 3.5 will bring it back down
      slide.setTargetPosition((int) (3.5 * TICKS_PER_REV));
      slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      slide.setPower(1.0);
    }
    if ( dumpState == 3 && ! slide.isBusy() ) {
      dumpState = 4;
      // After bringing the slide down using the encoder, we want to use the hardstop button to make sure we're actually at the bottom because the encoders aren't always perfect
      slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      slide.setPower(0.125);  // Just run the motor slowly...
    }
    if ( dumpState == 4 && hardstop.isPressed() ) { // Until the slide hits the hardstop button
      dumpState = -1;
      slide.setPower(0.0);
    }

    // We want to know when the play button is hit, but runtime.seconds() starts counting when you hit init
    // The first time we go through this loop, startTime will equal zero because that's what we declared it as at the top of the class
    // After that, startTime will have the value this line of code gave it during the first run-through
    if ( startTime == 0 ) startTime = runtime.seconds();

    // Reverse both intakes if:
    // You press the R1 or
    // Less than 1 second has passed since you hit play -> this deploys the secondary intake
    if ( gamepad1.right_bumper || runtime.seconds() - startTime <= 1.0 ) {
      intake.setPower(-1.0);
      secondIntake.setPower(1.0);
    } else {
      intake.setPower(1.0);
      secondIntake.setPower(-1.0);
    }
  }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Auto {
  // Variables that represent the motors, servos, sensors
  private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
  private Servo gate,bucketAngle;
  private TouchSensor hardstop;
  private BNO055IMU imu;
  private OpenCvCamera phoneCam;

  // General variables to be used later
  private ElapsedTime runtime = new ElapsedTime();
  private MegaPipeline pipeline;

  // The encoders in our motors tick 537.6 times per revolution
  // Our wheels are 96mm in diameter
  // A tile is 24 inches
  private final double TICKS_PER_REV = 537.6;
  private final double MM_PER_REV = Math.PI * 96;
  private final double MM_PER_TILE = 609.6;

  public void initAll(LinearOpMode op) {
    // When control is passed over to this class by one of the opmodes, the opmode gives us a reference to itself
    // This reference has useful variables like hardwareMap and sleep
    HardwareMap hardwareMap = op.hardwareMap;

    // Same as in teleop
    tlMotor     = hardwareMap.get(DcMotor.class,      "tl_motor");
    trMotor     = hardwareMap.get(DcMotor.class,      "tr_motor");
    blMotor     = hardwareMap.get(DcMotor.class,      "bl_motor");
    brMotor     = hardwareMap.get(DcMotor.class,      "br_motor");
    intake      = hardwareMap.get(DcMotor.class,      "intake");
    corner      = hardwareMap.get(DcMotor.class,      "corner");
    slide       = hardwareMap.get(DcMotor.class,      "slide");
    gate        = hardwareMap.get(Servo.class,        "gate");
    bucketAngle = hardwareMap.get(Servo.class,        "bucketAngle");
    hardstop    = hardwareMap.get(TouchSensor.class,  "hardstop");

    // Similar to teleop, except we're no longer using direct control over the drivetrain and slide motors - only encoder control
    tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    intake.setDirection(DcMotor.Direction.FORWARD);
    corner.setDirection(DcMotor.Direction.FORWARD);
    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    gate.setDirection(Servo.Direction.FORWARD);
    bucketAngle.setDirection(Servo.Direction.FORWARD);

    // This is how we set up the webcam with OpenCV
    // I copied and pasted most of this lol
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
    WebcamName webcamName = hardwareMap.get(WebcamName.class,"camera");
    phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);

    // In OpenCV you have to make a class (called a pipeline) that has a function where the webcam can send every frame it gets for processing
    pipeline = new MegaPipeline(op);
    phoneCam.setPipeline(pipeline);

    phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        phoneCam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode) {}
    });

    // This is how we set up the IMU
    // The IMU is a gyroscope that lives inside the control hub, and it lets us to fix our heading
    // Also copy and paste code
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.mode           = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);
    while ( ! op.isStopRequested() && ! imu.isGyroCalibrated() ) {
      op.sleep(50);
      op.idle();
    }

    // Just wait to make sure the camera has completely finished setting up
    op.sleep(2000);
  }

  // This is the function that the SAutoPathBL opmode runs
  // The function's responsibility is to get the robot in line and relatively close to the alliance hub (precision adjustments are then done with the camera)
  // Then it passes off control to blueDumpOnward, which is shared by both blueLeft and blueRight
  public void blueLeft(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    // We have to take the picture of the shipping element after, not before, we hit start, because if we take it after we hit init, the shipping element won't have been randomized
    // This function call is just to tell the camera to take the picture and find the element
    pipeline.getPicture();
    drive(-0.6);
    strafe(-1.075);
    drive(-0.15);
    blueDumpOnward(op);
  }

  public void blueRight(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    pipeline.getPicture();
    drive(-0.6);
    strafe(1.2);
    drive(-0.15);
    blueDumpOnward(op);
  }

  // Same as blueLeft and blueRight, just passes off to redDumpOnward instead
  public void redLeft(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    pipeline.getPicture();
    drive(-0.6);
    strafe(-1.15);
    drive(-0.15);
    redDumpOnward(op);
  }

  public void redRight(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    pipeline.getPicture();
    drive(-0.6);
    strafe(1.1);
    drive(-0.15);
    redDumpOnward(op);
  }

  private void blueDumpOnward(LinearOpMode op) {
      int region = pipeline.getConeRegion();  // It's still called a cone even though it's not a cone
      dump(op,region);

      // Push the robot up against the wall and drive it close to the duck carousel
      turnNinety(-1);
      strafe(0.5);
      drive(2.3,0.5);

      // Pass off control to blueDuckOnward
      blueDuckOnward(op);
  }

  private void blueDuckOnward(LinearOpMode op) {
    // To spin the duck carousel, we want to be constantly driving into it while spinning the corner
    // If you just drive up to it, stop, and spin the motor, you'll bounce off
      tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      corner.setPower(-1.0);  // Set corner to go
      tlMotor.setPower(-0.175); // Drive into the carousel slowly, left motors are backwards (we didn't set any direction like in teleop because we were using encoders in the beginning)
      trMotor.setPower(0.175);
      blMotor.setPower(-0.175);
      brMotor.setPower(0.175);
      op.sleep(5000);
      corner.setPower(0.0);
      tlMotor.setPower(0.0);
      trMotor.setPower(0.0);
      blMotor.setPower(0.0);
      brMotor.setPower(0.0);
      brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      // Drive into the warehouse
      drive(-1.0);
      strafe(0.6);
      drive(-3.5);
  }

  // Essentially the same as blueDumpOnward and blueDuckOnward
  private void redDumpOnward(LinearOpMode op) {
    int region = pipeline.getConeRegion();
    dump(op,region);

    drive(0.2);
    strafe(2.6,0.5);

    redDuckOnward(op);
  }

  private void redDuckOnward(LinearOpMode op) {
    tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    corner.setPower(1.0);
    tlMotor.setPower(-0.125);
    trMotor.setPower(0.135);
    blMotor.setPower(-0.125);
    brMotor.setPower(0.135);
    op.sleep(5000);
    corner.setPower(0.0);
    tlMotor.setPower(0.0);
    trMotor.setPower(0.0);
    blMotor.setPower(0.0);
    brMotor.setPower(0.0);
    brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    strafe(-1.0);
    turnNinety(-1);
    strafe(0.62);
    drive(3.65);
  }

  // This function is run by SAutoPathBLDuck to just do ducks and park, not dump
  // This is why we needed to split blueDumpOnward and blueDuckOnward – I want to do duckOnward without any dumping stuff
  // The rest of the duck functions are basically the same
  public void blueLeftDuck(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    // Drive near duck carousel
    drive(0.5);
    turnNinety(-1);
    drive(-3.6,0.75);
    blueDuckOnward(op);
  }

  public void blueRightDuck(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    drive(0.5);
    turnNinety(1);
    drive(-1.6,0.75);
    blueDuckOnward(op);
  }

  public void redLeftDuck(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    drive(-0.5);
    strafe(1.6,0.75);
    redDuckOnward(op);
  }

  public void redRightDuck(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    drive(-0.5);
    strafe(3.6,0.75);
    redDuckOnward(op);
  }

  // This function is run by SAutoPathBLDump to just dump and park, not do ducks
  // The rest of the dump functions are basically the same
  public void blueLeftDump(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    pipeline.getPicture();
    // Drive close to alliance hub
    drive(-0.6);
    strafe(-1.075);
    drive(-0.15);

    int region = pipeline.getConeRegion();
    dump(op,region);

    // Park in warehouse
    drive(0.5);
    turnNinety(1);
    strafe(-1.0,0.5);
    drive(2.3);
  }

  public void blueRightDump(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    pipeline.getPicture();
    drive(-0.6);
    strafe(1.275);
    drive(-0.15);

    int region = pipeline.getConeRegion();
    dump(op,region);

    drive(0.5);
    turnNinety(1);
    strafe(-1.0,0.5);
    drive(2.3);
  }

  public void redLeftDump(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    pipeline.getPicture();
    drive(-0.6);
    strafe(-1.15);
    drive(-0.15);

    int region = pipeline.getConeRegion();
    dump(op,region);

    drive(0.5);
    turnNinety(-1);
    strafe(1.0,0.5);
    drive(2.3);
  }

  public void redRightDump(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    pipeline.getPicture();
    drive(-0.6);
    strafe(1.1);
    drive(-0.15);

    int region = pipeline.getConeRegion();
    dump(op,region);

    drive(0.5);
    turnNinety(-1);
    strafe(1.0,0.5);
    drive(2.3);
  }

  // This function is run by SAutoPathBLPark to just park
  // The simplest function, it literally just parks, and the rest of the park functions are basically the same
  public void blueLeftPark(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    drive(-0.5);
    turnNinety(1);
    strafe(-1.0,0.5);
    drive(1.3);
  }

  public void blueRightPark(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    drive(-0.5);
    turnNinety(1);
    strafe(-1.0,0.5);
    drive(3.3);
  }

  public void redLeftPark(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    drive(-0.5);
    turnNinety(-1);
    strafe(1.0,0.5);
    drive(3.3);
  }

  public void redRightPark(HardwareMap hardwareMap,LinearOpMode op) {
    initAll(op);
    op.waitForStart();
    drive(-0.5);
    turnNinety(-1);
    strafe(1.0,0.5);
    drive(1.3);
  }

  private void dump(LinearOpMode op,int region) {
    // We're turning off encoders bc we're driving towards the alliance hub until we see a certain average darkness from the camera (coming from the black of the hub)
    brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    double start = runtime.seconds();
    while ( pipeline.getAvgD() > 100 && runtime.seconds() - start < 0.25 ) {  // Go towards the hub until the camera thinks the view is dark enough or 0.25 seconds have passed (failsafe)
      tlMotor.setPower(0.2);
      trMotor.setPower(-0.2);
      blMotor.setPower(0.2);
      brMotor.setPower(-0.2);
    }
    // And then stop and go back to using encoders
    tlMotor.setPower(0);
    trMotor.setPower(0);
    blMotor.setPower(0);
    brMotor.setPower(0);
    tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    op.sleep(500);

    // After pulling up to the hub, pull back to land the box in the correct level of the hub based on the region of the shipping element
    // Left/bottom level is zero, middle is one, right/top level is two
    if ( region == 1 ) drive(0.1);
    else if ( region == 0 ) drive(0.175);

    bucketAngle.setPosition(0.05);
    // Lift the slide 3 revolutions and wait until its done
    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide.setTargetPosition((int) (-3.0 * TICKS_PER_REV));
    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slide.setPower(0.5);
    while ( slide.isBusy() ) {}
    // Dump the bucket
    bucketAngle.setPosition(1.0);
    op.sleep(1000);
    bucketAngle.setPosition(0.0);
    op.sleep(1000);
    // Put the slide down 3 revolutions and wait until its done
    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide.setTargetPosition((int) (3.0 * TICKS_PER_REV));
    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slide.setPower(0.5);
    while ( slide.isBusy() ) {}
    // Go down slowly until hardstop button pressed
    slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    slide.setPower(0.125);
    while ( ! hardstop.isPressed() ) {}
    // Then stop the slide
    slide.setPower(0.0);
  }

  // If you don't specify a speed value, it's assumed to be full speed
  private void drive(double distance) { drive(distance,1); }
  private void strafe(double distance) { strafe(distance,1); }

  private void drive(double distance,double speed) {
    moveMotors(distance,distance,distance,distance,speed); // Obviously
  }

  private void strafe(double distance,double speed) { // left is negative, right is positive
    moveMotors(distance,-distance,-distance,distance,speed);  // Negative signs found by guessing
  }

  private void turnNinety(int turns) {
      moveMotors(0.825 * turns,-0.825 * turns,0.825 * turns,-0.825 * turns,1);  // 0.825 found by guessing
  }

  // Literally the most critical function in all the code so pay attention
  private void moveMotors(double tlMove,double trMove,double blMove,double brMove,double speed) {
      tlMotor.setTargetPosition((int) (-tlMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));  // Remember how the left motors are reversed
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
      while ( tlMotor.isBusy() || trMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy() ) {} // Wait until all the motors are done moving (stay in the loop if any of them are still busys)
      // Stop all the motors
      brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  // The pipeline class that OpenCV uses for the camera
  class MegaPipeline extends OpenCvPipeline {
    final double FRAME_WIDTH = 320;
    final double FRAME_HEIGHT = 240;

    // For purposes of shipping element identification, it's on the left if its between 0 and REGION_0_END,
    // in the middle between REGION_0_END and REGION_1_END, and on the right from REGION_1_END onward
    // Setting REGION_0_END to 1/3 of the way across the screen and REGION_1_END to 2/3 worked, but it could be changed if the camera was off-center
    final double REGION_0_END = (FRAME_WIDTH / 3);
    final double REGION_1_END = (2 * FRAME_WIDTH / 3);

    private ElapsedTime runtime = new ElapsedTime();
    private boolean firstFrame = false;
    private int coneRegion = -1;
    private int avgD = 0;
    private LinearOpMode opa;
    private boolean doGetPicture = false;

    public MegaPipeline(LinearOpMode opa) {
      // Just like all the other functions that take op as a parameter, the pipeline needs access to the special opmode variables and functions (like logging)
      this.opa = opa;
    }

    @Override
    public Mat processFrame(Mat input) {
      // Calculate where the shipping element is only if:
      // We got the instruction to "getPicture," which we get after the play button is pressed
      // At least one second has passed since the pipeline started - when the camera turns on, the exposure is way off but then it auto-adjusts within a second
      // We haven't already calculated that
      if ( doGetPicture && runtime.seconds() > 1.0 && ! firstFrame ) {
        // Set firstFrame to true so we never again try to recalculate where the shipping element is
        // If we tried to recalculate after we already moved that would be bad
        firstFrame = true;

        double r0sum = 0;
        double r1sum = 0;
        double r2sum = 0;

        for ( int x = 0; x < REGION_0_END; x++ ) {
          for ( int y = (int) FRAME_HEIGHT / 2; y < FRAME_HEIGHT; y++ ) {
            // Each region is given a score of how much of the shipping element it probably contains
            // That's calculated by taking each pixel in the region and adding to the region score the value (green - (red + blue) / 2)
            // The RGB values go from 0 to 255
            double[] rgb = input.get(y,x);
            r0sum += rgb[1] - (rgb[0] + rgb[2]) / 2;
          }
        }
        // Same as the region 0 loop, but for regions 1 and 2
        for ( int x = (int) REGION_0_END; x < REGION_1_END; x++ ) {
          for ( int y = (int) FRAME_HEIGHT / 2; y < FRAME_HEIGHT; y++ ) {
            double[] rgb = input.get(y,x);
            r1sum += rgb[1] - (rgb[0] + rgb[2]) / 2;
          }
        }
        for ( int x = (int) REGION_1_END; x < FRAME_WIDTH; x++ ) {
          for ( int y = (int) FRAME_HEIGHT / 2; y < FRAME_HEIGHT; y++ ) {
            double[] rgb = input.get(y,x);
            r2sum += rgb[1] - (rgb[0] + rgb[2]) / 2;
          }
        }

        // Get the region with the maximum sum
        // wow you do this in AP Java!!!!
        if ( r0sum > r1sum ) {
          if ( r0sum > r2sum ) coneRegion = 0;
          else coneRegion = 2;
        } else {
          if ( r1sum > r2sum ) coneRegion = 1;
          else coneRegion = 2;
        }

        // This section should be like loops and stuff instead of three copy-pasted things but it's only three regions~
      }

      // Convert the frame from the webcam from RGB format to HSV
      // HSV is hue-saturation-value
      // Instead of each pixel being encoded as how red, green, and blue it is, each pixel is represented as what color it is, how saturated the color is, and how light or dark it is (value)
      Mat hsvMat = new Mat();
      Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);

      // Calculate the average darkness (aka HSV value) (really it's brightness, a lower darkness number means the frame is darker) for the entire frame
      // Get the HSV value of each pixel, add them all up, and divide by the amount of pixels in the frame
      long totalD = 0;
      for ( int x = 0; x < FRAME_WIDTH; x++ ) {
          for ( int y = 0; y < FRAME_HEIGHT; y++ ) {
              double hsv[] = hsvMat.get(y,x);
              totalD += hsv[2];
          }
      }
      avgD = (int) (totalD / (FRAME_WIDTH * FRAME_HEIGHT));

      return input;
    }

    @Override
    public void onViewportTapped() {}

    // Return the calculated cone region when some other function needs it
    // Maybe I could've just made the coneRegion variable public, but this was recommended (and more correct I guess)
    public int getConeRegion() {
      return coneRegion;
    }

    // Return the calculated average darkness
    public int getAvgD() {
      return avgD;
    }

    // Make the pipeline calculate the cone region on the next frame it receives
    public void getPicture() {
      doGetPicture = true;
    }
  }

  // These are some extra modes I wrote at the qualifier
  // Challenge: figure out what they do, draw their paths on paper
  public void redQualifier(HardwareMap hardwareMap,LinearOpMode op) {
      initAll(op);
      op.waitForStart();
      drive(0.5);
      turnNinety(2);
      strafe(2,0.5);

      tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      corner.setPower(1.0);
      tlMotor.setPower(-0.125);
      trMotor.setPower(0.135);
      blMotor.setPower(-0.125);
      brMotor.setPower(0.135);
      op.sleep(5000);
      corner.setPower(0.0);
      tlMotor.setPower(0.0);
      trMotor.setPower(0.0);
      blMotor.setPower(0.0);
      brMotor.setPower(0.0);
      brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      drive(-0.75);
  }

  public void blueQualifier(HardwareMap hardwareMap,LinearOpMode op) {
      initAll(op);
      op.waitForStart();
      drive(0.5);
      turnNinety(1);
      drive(2,0.5);

      tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      corner.setPower(-1.0);
      tlMotor.setPower(-0.125);
      trMotor.setPower(-0.125);
      blMotor.setPower(0.125);
      brMotor.setPower(0.125);
      op.sleep(7000);
      corner.setPower(0.0);
      tlMotor.setPower(0.0);
      trMotor.setPower(0.0);
      blMotor.setPower(0.0);
      brMotor.setPower(0.0);
      brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      strafe(-1);
  }

  public void blueQualifierWithoutDucks(HardwareMap hardwareMap,LinearOpMode op) {
      initAll(op);
      op.waitForStart();
      drive(0.5);
      turnNinety(1);
      drive(2,0.5);
      strafe(-0.7);
  }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
    private Servo gate,bucketAngle;
    private TouchSensor hardstop;
    private OpenCvCamera phoneCam;
    private ElapsedTime runtime = new ElapsedTime();

    private final double TICKS_PER_REV = 537.6;
    private final double MM_PER_REV = Math.PI * 96;
    private final double MM_PER_TILE = 609.6;

    public void initAll(HardwareMap hardwareMap) {
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
    }

    public void redLeft(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = getConeRegion(hardwareMap,op);
        strafe(1.25);
        redCore(hardwareMap,op,coneRegion);
    }

    public void redRight(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = getConeRegion(hardwareMap,op);
        strafe(-1.15);
        redCore(hardwareMap,op,coneRegion);
    }

    public void blueLeft(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = 2;//getConeRegion(hardwareMap,op);
        strafe(1.0);
        blueCore(hardwareMap,op,coneRegion);
    }

    public void blueRight(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = getConeRegion(hardwareMap,op);
        strafe(-1.275);
        blueCore(hardwareMap,op,coneRegion);
    }

    public void redLeftDuck(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = getConeRegion(hardwareMap,op);
        drive(0.5);
        turnNinety(2);
        strafe(1.6,0.75);
        redDuckOnward(hardwareMap,op,coneRegion);
    }

    public void redRightDuck(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = getConeRegion(hardwareMap,op);
        drive(0.5);
        turnNinety(2);
        strafe(3.6,0.75);
        redDuckOnward(hardwareMap,op,coneRegion);
    }

    public void blueLeftDuck(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = getConeRegion(hardwareMap,op);
        drive(0.5);
        turnNinety(1);
        drive(-3.6,0.75);
        blueDuckOnward(hardwareMap,op,coneRegion);
    }

    public void blueRightDuck(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        int coneRegion = getConeRegion(hardwareMap,op);
        drive(0.5);
        turnNinety(1);
        drive(-1.6,0.75);
        blueDuckOnward(hardwareMap,op,coneRegion);
    }

    public void redLeftPark(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        turnNinety(1);
        strafe(-0.5,0.2);
        drive(3.3);
    }

    public void redRightPark(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        turnNinety(1);
        strafe(-0.5,0.2);
        drive(1.3);
    }

    public void blueLeftPark(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        turnNinety(-1);
        strafe(-0.5,0.2);
        drive(1.3);
    }

    public void blueRightPark(HardwareMap hardwareMap,LinearOpMode op) {
        initAll(hardwareMap);
        op.waitForStart();
        turnNinety(-1);
        strafe(-0.5,0.2);
        drive(3.3);
    }

    private void redCore(HardwareMap hardwareMap,LinearOpMode op,int coneRegion) {
        double revolutions;
        if ( coneRegion <= 0 ) revolutions = 4;
        else revolutions = 2.95;

        drive(0.75);
        turnNinety(2);

        bucketAngle.setPosition(0.05);
        if ( coneRegion == 2 ) drive(0.15);
        slide.setTargetPosition((int) (-revolutions * TICKS_PER_REV));
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        while ( slide.isBusy() ) {}
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketAngle.setPosition(1.0);
        op.sleep(1000);
        bucketAngle.setPosition(0.0);
        op.sleep(500);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(0.125);
        while ( ! hardstop.isPressed() ) {}
        slide.setPower(0.0);
        if ( coneRegion == 2 ) drive(-0.15);

        drive(0.5);
        strafe(2.6,0.75);

        redDuckOnward(hardwareMap,op,coneRegion);
    }

    private void redDuckOnward(HardwareMap hardwareMap,LinearOpMode op,int coneRegion) {
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

        drive(3.5);
    }

    private void blueCore(HardwareMap hardwareMap,LinearOpMode op,int coneRegion) {
        double revolutions;
        if ( coneRegion <= 0 ) revolutions = 4;
        else revolutions = 2.95;

        drive(0.75);
        turnNinety(2);

        bucketAngle.setPosition(0.05);
        if ( coneRegion == 2 ) drive(0.15);
        slide.setTargetPosition((int) (-revolutions * TICKS_PER_REV));
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        while ( slide.isBusy() ) {}
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketAngle.setPosition(1.0);
        op.sleep(1000);
        bucketAngle.setPosition(0.0);
        op.sleep(500);
        if ( coneRegion == 2 ) drive(-0.15);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(0.25);
        while ( ! hardstop.isPressed() ) {}
        slide.setPower(0.0);

        turnNinety(-1);
        strafe(0.5);
        drive(2.3,0.75);

        blueDuckOnward(hardwareMap,op,coneRegion);
    }

    private void blueDuckOnward(HardwareMap hardwareMap,LinearOpMode op,int coneRegion) {
        tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        corner.setPower(-1.0);
        tlMotor.setPower(-0.125);
        trMotor.setPower(-0.125);
        blMotor.setPower(0.125);
        brMotor.setPower(0.125);
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

        drive(-1.0);
        strafe(0.6);
        //strafe(-0.05); // To compensate for edges of mat

        drive(-3.3);
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

    public int getConeRegion(HardwareMap hardwareMap,LinearOpMode op) {
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
      phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

      ConePipeline pipeline = new ConePipeline();
      phoneCam.setPipeline(pipeline);

      phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
          phoneCam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {}
      });

      while ( pipeline.getConeRegion() == -1 && runtime.seconds() < 4 ) {}
      op.telemetry.addData("coneRegion",pipeline.getConeRegion());
      op.telemetry.update();
      return pipeline.getConeRegion();
    }

    class ConePipeline extends OpenCvPipeline {
      final double FRAME_WIDTH = 320;
      final double FRAME_HEIGHT = 240;
      final double REGION_0_END = (FRAME_WIDTH / 3);
      final double REGION_1_END = (2 * FRAME_WIDTH / 3);

      private ElapsedTime runtime = new ElapsedTime();
      private boolean processing = false;
      private int coneRegion = -1;

      @Override
      public Mat processFrame(Mat input) {
        Imgproc.line(
          input,
          new Point(REGION_0_END,0),
          new Point(REGION_0_END,FRAME_HEIGHT),
          new Scalar(255,0,0),
          1
        );
        Imgproc.line(
          input,
          new Point(REGION_1_END,0),
          new Point(REGION_1_END,FRAME_HEIGHT),
          new Scalar(255,0,0),
          1
        );
        Imgproc.line(
          input,
          new Point(0,FRAME_HEIGHT / 2),
          new Point(FRAME_WIDTH,FRAME_HEIGHT / 2),
          new Scalar(255,0,0),
          1
        );

        if ( runtime.seconds() <= 1.0 ) return input;
        if ( processing ) return input;
        processing = true;

        double r0sum = 0;
        double r1sum = 0;
        double r2sum = 0;

        for ( int x = 0; x < REGION_0_END; x++ ) {
          for ( int y = (int) FRAME_HEIGHT / 2; y < FRAME_HEIGHT; y++ ) {
            double[] rgb = input.get(y,x);
            r0sum += rgb[1] - rgb[0] - rgb[2];
          }
        }
        for ( int x = (int) REGION_0_END; x < REGION_1_END; x++ ) {
          for ( int y = (int) FRAME_HEIGHT / 2; y < FRAME_HEIGHT; y++ ) {
            double[] rgb = input.get(y,x);
            r1sum += rgb[1] - rgb[0] - rgb[2];
          }
        }
        for ( int x = (int) REGION_1_END; x < FRAME_WIDTH; x++ ) {
          for ( int y = (int) FRAME_HEIGHT / 2; y < FRAME_HEIGHT; y++ ) {
            double[] rgb = input.get(y,x);
            r2sum += rgb[1] - rgb[0] - rgb[2];
          }
        }

        if ( r0sum > r1sum ) {
          if ( r0sum > r2sum ) coneRegion = 0;
          else coneRegion = 2;
        } else {
          if ( r1sum > r2sum ) coneRegion = 1;
          else coneRegion = 2;
        }
        coneRegion = 3 - coneRegion; // 2 is left, 1 is middle, 0 is right

        return input;
      }

      @Override
      public void onViewportTapped() {}

      public int getConeRegion() {
        return coneRegion;
      }
    }
}

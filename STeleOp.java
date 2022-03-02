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
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide,secondIntake;
    private BNO055IMU imu;
    private Servo gate,bucketAngle;
    private TouchSensor hardstop;
    private OpenCvCamera phoneCam;
    private ElapsedTime runtime = new ElapsedTime();
    private int dumpState = -1;
    private double dumpTime;
    private ConePipeline pipeline;

    private final double TICKS_PER_REV = 537.6;
    private final double MM_PER_REV = Math.PI * 96;
    private final double MM_PER_TILE = 609.6;

    @Override
    public void init() {
      tlMotor     = hardwareMap.get(DcMotor.class,  "tl_motor");
      trMotor     = hardwareMap.get(DcMotor.class,  "tr_motor");
      blMotor     = hardwareMap.get(DcMotor.class,  "bl_motor");
      brMotor     = hardwareMap.get(DcMotor.class,  "br_motor");
      intake      = hardwareMap.get(DcMotor.class,  "intake");
      corner      = hardwareMap.get(DcMotor.class,  "corner");
      slide       = hardwareMap.get(DcMotor.class,  "slide");
      secondIntake       = hardwareMap.get(DcMotor.class,  "secondIntake");
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

      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

      parameters.mode                = BNO055IMU.SensorMode.IMU;
      parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
      parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      parameters.loggingEnabled      = false;

      imu = hardwareMap.get(BNO055IMU.class,"imu");
      imu.initialize(parameters);
      while ( ! imu.isGyroCalibrated() ) {
        try {
          TimeUnit.MILLISECONDS.sleep(50);
        } catch ( InterruptedException e ) {}
      }

      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
      phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

      pipeline = new ConePipeline();
      phoneCam.setPipeline(pipeline);

      phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
          phoneCam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {}
      });
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
        slide.setTargetPosition((int) (-3.5 * TICKS_PER_REV));
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
        slide.setTargetPosition((int) (3.5 * TICKS_PER_REV));
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

      if ( gamepad1.x ) {
        strafe(-2 + (6.5/24),1);
        double start = runtime.seconds();
        while ( pipeline.getAvgD() < 8000 && runtime.seconds() - start < 0.25 ) {
            telemetry.addData("avgD",pipeline.getAvgD());
            telemetry.update();
            tlMotor.setPower(-0.2);
            trMotor.setPower(-0.2);
            blMotor.setPower(-0.2);
            brMotor.setPower(-0.2);
        }
        telemetry.addData("avgD",pipeline.getAvgD());
        telemetry.update();
        tlMotor.setPower(0);
        trMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
      }

      if ( ! gamepad1.right_bumper ) {
        //intake.setPower(1.0);
        //secondIntake.setPower(1.0);
      } else {
        //intake.setPower(-1.0);
        //secondIntake.setPower(-1.0);
      }
    }

    private void strafe(double distance,double speed) { // left is negative, right is positive
        distance *= 1.15;
        moveMotors(distance,-distance,-distance,distance,speed);
    }

    private void moveMotors(double tlMove,double trMove,double blMove,double brMove,double speed) {
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tlMotor.setTargetPosition((int) (tlMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));
        trMotor.setTargetPosition((int) (trMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));
        blMotor.setTargetPosition((int) (blMove * MM_PER_TILE / MM_PER_REV * TICKS_PER_REV));
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
        tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tlMotor.setPower(0);
        trMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        Orientation angles;
        double delta = 0.0;
        double startTime = runtime.seconds();
        do {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            delta = angles.firstAngle;
            tlMotor.setPower(delta * 0.05);
            trMotor.setPower(-delta * 0.05);
            blMotor.setPower(delta * 0.05);
            brMotor.setPower(-delta * 0.05);
        } while ( Math.abs(delta) > 1 && runtime.seconds() - startTime < 0.5 );
        tlMotor.setPower(0);
        trMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
        secondIntake.setPower(0.0);
    }

    class DarkPipeline extends OpenCvPipeline {
        final double FRAME_WIDTH = 320;
        final double FRAME_HEIGHT = 240;

        private ElapsedTime runtime = new ElapsedTime();
        private boolean processing = false;
        private int avgD = -1;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);

            avgD = 0;
            for ( int x = 0; x < FRAME_WIDTH; x++ ) {
                for ( int y = (int) (5 * FRAME_HEIGHT / 6); y < FRAME_HEIGHT; y++ ) {
                    double hsv[] = hsvMat.get(y,x);
                    if ( hsv[2] < 40 ) {
                      avgD++;
                      hsv[0] = 100;
                      hsv[1] = 255;
                      hsv[2] = 255;
                    } else {
                      //hsv[0] = 100;
                      //hsv[1] = 255;
                      //hsv[2] = 255;
                    }
                    hsvMat.put(y,x,hsv);
                }
            }

            Imgproc.cvtColor(hsvMat,input,Imgproc.COLOR_HSV2RGB);
            return input;
        }

        @Override
        public void onViewportTapped() {}

        public int getAvgD() {
          return avgD;
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name="SAutoTesting", group="Linear Opmode")
public class SAutoTesting extends LinearOpMode {
    private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
    private Servo gate,bucketAngle;
    private TouchSensor hardstop;
    private BNO055IMU imu;
    private OpenCvCamera phoneCam;
    private ElapsedTime runtime = new ElapsedTime();

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        DarkPipeline pipeline = new DarkPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while ( ! isStopRequested() && ! imu.isGyroCalibrated() ) {
            sleep(50);
            idle();
        }

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

        drive(-0.75,1);
        strafe(-1,1);
        drive(-0.1,1);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double start = runtime.seconds();
        while ( pipeline.getAvgD() > 100 && runtime.seconds() - start < 0.5 ) {
            telemetry.addData("avgD",pipeline.getAvgD());
            telemetry.update();
            tlMotor.setPower(0.2);
            trMotor.setPower(-0.2);
            blMotor.setPower(0.2);
            brMotor.setPower(-0.2);
        }
        telemetry.addData("avgD",pipeline.getAvgD());
        telemetry.update();
        tlMotor.setPower(0);
        trMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        bucketAngle.setPosition(0.05);
        slide.setTargetPosition((int) (-2.5 * TICKS_PER_REV));
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        while ( slide.isBusy() ) {}
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketAngle.setPosition(1.0);
        sleep(1000);
        bucketAngle.setPosition(0.0);
        sleep(500);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(0.125);
        while ( ! hardstop.isPressed() ) {}
        slide.setPower(0.0);
    }

    private void drive(double distance) { drive(distance,1); }
    private void strafe(double distance) { strafe(distance,1); }

    private void drive(double distance,double speed) {
        moveMotors(distance,distance,distance,distance,speed);
    }

    private void strafe(double distance,double speed) { // left is negative, right is positive
        distance *= 1.15;
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
        /*Orientation angles;
        double delta = 0.0;
        double startTime = runtime.seconds();
        do {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            delta = angles.firstAngle;
            tlMotor.setPower(delta * 0.05);
            trMotor.setPower(-delta * 0.05);
            blMotor.setPower(delta * 0.05);
            brMotor.setPower(-delta * 0.05);
        } while ( Math.abs(delta) > 0.5 && runtime.seconds() - startTime < 0.5 );
        tlMotor.setPower(0);
        trMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);*/
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

            long totalV = 0;
            for ( int x = 0; x < FRAME_WIDTH; x++ ) {
                for ( int y = 0; y < FRAME_HEIGHT; y++ ) {
                    double hsv[] = hsvMat.get(y,x);
                    totalV += hsv[2];
                }
            }
            avgD = (int) (totalV / (FRAME_WIDTH * FRAME_HEIGHT));

            return input;
        }

        @Override
        public void onViewportTapped() {}

        public int getAvgD() {
          return avgD;
        }
    }
}

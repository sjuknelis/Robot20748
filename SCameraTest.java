package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class SCameraTest extends LinearOpMode {
  OpenCvCamera phoneCam;
  private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
  private Servo gate,bucketAngle;
  private TouchSensor hardstop;

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
    hardstop    = hardwareMap.get(TouchSensor.class,"hardstop");

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

    waitForStart();

    while ( pipeline.getConeRegion() == -1 ) {}
    telemetry.addData("coneRegion",pipeline.getConeRegion());
    telemetry.update();
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
      telemetry.addData("r0sum",r0sum);
      telemetry.addData("r1sum",r1sum);
      telemetry.addData("r2sum",r2sum);

      if ( r0sum > r1sum ) {
        if ( r0sum > r2sum ) coneRegion = 0;
        else coneRegion = 2;
      } else {
        if ( r1sum > r2sum ) coneRegion = 1;
        else coneRegion = 2;
      }

      return input;
    }

    @Override
    public void onViewportTapped() {}

    public int getConeRegion() {
      return coneRegion;
    }
  }
}

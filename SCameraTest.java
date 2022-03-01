package org.firstinspires.ftc.teamcode;

import java.util.*;

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

@TeleOp
public class SCameraTest extends LinearOpMode {
  OpenCvCamera phoneCam;
  private DcMotor tlMotor,trMotor,blMotor,brMotor,intake,corner,slide;
  private Servo gate,bucketAngle;
  private TouchSensor hardstop;
  private ElapsedTime runtime;

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

    tlMotor.setDirection(DcMotor.Direction.REVERSE);
    trMotor.setDirection(DcMotor.Direction.FORWARD);
    blMotor.setDirection(DcMotor.Direction.REVERSE);
    brMotor.setDirection(DcMotor.Direction.FORWARD);
    intake.setDirection(DcMotor.Direction.FORWARD);
    corner.setDirection(DcMotor.Direction.FORWARD);
    slide.setDirection(DcMotor.Direction.FORWARD);
    gate.setDirection(Servo.Direction.FORWARD);
    bucketAngle.setDirection(Servo.Direction.FORWARD);

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

    runtime = new ElapsedTime();
    while ( pipeline.getAvgX() == -1 || runtime.seconds() >= 20.0 ) {}
    while ( Math.abs(pipeline.getAvgX() - 160) > 5 ) {
      int sign = (int) Math.signum(pipeline.getAvgX() - 160);
      tlMotor.setPower(sign * 0.25);
      trMotor.setPower(sign * -0.25);
      blMotor.setPower(sign * 0.25);
      brMotor.setPower(sign * -0.25);
    }
  }

  class ConePipeline extends OpenCvPipeline {
    final double FRAME_WIDTH = 320;
    final double FRAME_HEIGHT = 240;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean processing = false;
    private int avgX = -1;
    private int avgY = -1;

    @Override
    public Mat processFrame(Mat input) {
      //if ( processing ) return input;
      //processing = true;

      Mat hsvMat = new Mat();
      Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);

      for ( int x = 0; x < FRAME_WIDTH; x++ ) {
        for ( int y = 0; y < FRAME_HEIGHT; y++ ) {
          double hsv[] = hsvMat.get(y,x);
          if ( ! ((hsv[0] < 20 || hsv[0] > 160) && hsv[1] > 150) ) hsv[2] = 0;
          else {
            hsv[1] = 0;
            hsv[2] = 255;
          }
          hsvMat.put(y,x,hsv);
        }
      }

      Mat gray = new Mat();
      Imgproc.cvtColor(hsvMat,gray,Imgproc.COLOR_HSV2RGB);
      Imgproc.cvtColor(gray,gray,Imgproc.COLOR_RGB2GRAY);

      List<MatOfPoint> contours = new ArrayList<>();
      Mat hierarchy = new Mat();
      Imgproc.findContours(gray,contours,hierarchy,Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
      for ( int i = 0; i < contours.size(); i++ ) {
        Imgproc.drawContours(hsvMat,contours,i,new Scalar(0,0,0),1);
        Imgproc.drawContours(gray,contours,i,new Scalar(0),1);
      }

      contours = new ArrayList<>();
      hierarchy = new Mat();
      Imgproc.findContours(gray,contours,hierarchy,Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
      for ( int i = 0; i < contours.size(); i++ ) {
        if ( Imgproc.contourArea(contours.get(i)) > 5 ) Imgproc.drawContours(hsvMat,contours,i,new Scalar(100,100,100),10);
      }

      long totalX = 0;
      long totalY = 0;
      int totalCount = 0;
      for ( int x = 0; x < FRAME_WIDTH; x++ ) {
        for ( int y = 0; y < FRAME_HEIGHT; y++ ) {
          double hsv[] = hsvMat.get(y,x);
          if ( hsv[0] == 100 ) {
            totalX += x;
            totalY += y;
            totalCount++;
          }
        }
      }
      avgX = (int) (totalX / totalCount);
      avgY = (int) (totalY / totalCount);
      Imgproc.rectangle(
        hsvMat,
        new Point(avgX - 10,avgY - 10),
        new Point(avgX + 10,avgY + 10),
        new Scalar(40,255,255),
        2
      );
      telemetry.addData("avgX",avgX);
      telemetry.update();

      Imgproc.cvtColor(hsvMat,input,Imgproc.COLOR_HSV2RGB);
      return input;
    }

    @Override
    public void onViewportTapped() {}

    public int getAvgX() {
      return avgX;
    }
  }
}

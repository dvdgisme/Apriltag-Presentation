/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.PowerPlay2023Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class NewLeft extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private Servo claw;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private Servo wrist;
    private DcMotor elbow;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag ID 0, 1, 2 from the 36h11 family
    int LEFT = 2;
    int MIDDLE = 0;
    int RIGHT = 1;

    AprilTagDetection tagOfInterest = null;

    int leftPos;
    int rightPos;

    @Override
    public void runOpMode() {

        double wristPos;
        double elbowPos;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        claw = hardwareMap.get(Servo.class, "claw");
        leftArm = hardwareMap.get(DcMotor.class, "Left Arm");
        rightArm = hardwareMap.get(DcMotor.class, "Right Arm");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw.setPosition(1);

        elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                /**
                 * Finished here in the video titled "6547's FTC PowerPlay Tutorial for Camera Vision (AprilTags)"
                 */

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        telemetry.addData(">>>", "Press Play To Start");
        waitForStart();
        if (opModeIsActive()) {
            elbowPos = 1.0;
            wristPos = 1.0;
            //ArmUp initial position
            wrist.setPosition(wristPos);
            sleep(100);
            int ArmUpInit = 1000;
            //Display what is active
            String active = "";
                if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                    active += "Left, 1 Dot, ";

                    //Back up to the cones
                    elbow.setPower(-0.4);
                    DriveBackwardDistance(0.5,400);
                    sleep(100);

                    //lift the arm
                    Arm(1);
                    sleep(1400);
                    Arm(0);
                    DriveBackwardDistance(0.5,1600);
                    sleep(1000);

                    //Align with the pole backwards
                    Turn(0.5,-800,-800,-800,-800);
                    sleep(1000);
                    DriveBackwardDistance(.5,100);

                    elbow.setPower(0);
                    sleep(50);
                    //Set the elbow to the correct scoring position
                    elbow.setPower(.5);
                    sleep(500);
                    elbow.setPower(0.1);
                    sleep(50);
                    wrist.setPosition(0.74);
                    //Give time for the servos to extend
                    sleep(1000);

                    //Open the claw to drop the cone
                    claw.setPosition(0.35);
                    sleep(200);

                    //go into spot 1
                    Turn(0.5,320,320,320,320);
                    sleep(1000);
                    Arm(0);
                    elbow.setPower(0);
                    DriveBackwardDistance(0.5,1000);
                    claw.setPosition(0);



                    telemetry.addData(">>>Active: ", active);
                    telemetry.update();
                } else if (tagOfInterest.id == MIDDLE) {
                    active += "Middle, 2 Dots, ";
                    DriveBackwardDistance(0.5,400);
                    Arm(1);
                    sleep(1400);
                    Arm(0);
                    DriveBackwardDistance(0.5,1600);
                    sleep(1000);
                    //Align with the pole
                    Turn(0.5,600,600,600,600);
                    sleep(1000);
                    //Drive forward to give the arm room to extend
                    DriveForwardDistance(0.5,400);
                    //Run arm to scoring position

                    //Back up to pole
                    DriveBackwardDistance(.5,500);
                    //Turn towards the pole
                    Turn(0.5,-220,-220,-220,-220);
                    //Set the servos to the correct scoring position
                    elbow.setPower(.5);
                    wrist.setPosition(0.3);
                    //Give time for the servos to extend
                    sleep(1000);
                    elbow.setPower(0);
                    sleep(400);
                    //Open the claw to drop the cone
                    claw.setPosition(0.35);
                    sleep(200);
                    //go into spot 1
                    Turn(0.5,320,320,320,320);
                    sleep(1000);
                    Arm(0);
                    elbow.setPower(0);
                    DriveForwardDistance(0.5,-500);
                    claw.setPosition(0);


                    telemetry.addData(">>>Active: ", active);
                    telemetry.update();
                } else /* tagOfInterest.id == RIGHT */ {
                    active += "Right, 3 Dots, ";
                    DriveBackwardDistance(0.5,400);
                    Arm(1);
                    sleep(1400);
                    Arm(0);
                    DriveBackwardDistance(0.5,1600);
                    sleep(1000);
                    //Align with the pole
                    Turn(0.5,600,600,600,600);
                    sleep(1000);
                    //Drive forward to give the arm room to extend
                    DriveForwardDistance(0.5,400);
                    //Run arm to scoring position

                    //Back up to pole
                    DriveBackwardDistance(.5,500);
                    //Turn towards the pole
                    Turn(0.5,-220,-220,-220,-220);
                    //Set the servos to the correct scoring position
                    elbow.setPower(.5);
                    wrist.setPosition(0.3);
                    //Give time for the servos to extend
                    sleep(1000);
                    elbow.setPower(0);
                    sleep(400);
                    //Open the claw to drop the cone
                    claw.setPosition(0.35);
                    sleep(200);
                    //go into spot 1
                    Turn(0.5,320,320,320,320);
                    sleep(1000);
                    Arm(0);
                    elbow.setPower(0);
                    DriveForwardDistance(0.5,-1000);
                    claw.setPosition(0);


                    telemetry.addData(">>>Active: ", active);
                    telemetry.update();

            }

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void endPower() {
        leftArm.setPower(0);
        rightArm.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(100);
    }

    public void Drive(double power){
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        leftRear.setPower(power);

    }

    public void Arm(double power){
        leftArm.setPower(power);
        rightArm.setPower(power);
    }

    public void StopDriving(){
        Drive(0);
    }
    public void StopArm(){
        Arm(0);
    }

    public void DriveForwardDistance(double power, int distance){
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setTargetPosition(-distance);
        rightRear.setTargetPosition(-distance);
        leftFront.setTargetPosition(distance);
        leftRear.setTargetPosition(distance);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(power);

        while(rightFront.isBusy() && rightRear.isBusy() && leftFront.isBusy() && leftRear.isBusy()){

        }
        StopDriving();
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    public void DriveBackwardDistance(double power, int distance){
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);
        leftFront.setTargetPosition(-distance);
        leftRear.setTargetPosition(-distance);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(power);

        while(rightFront.isBusy() && rightRear.isBusy() && leftFront.isBusy() && leftRear.isBusy()){

        }
        StopDriving();
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void Turn(double power, int LeftFront, int RightFront, int LeftRear, int RightRear){
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /**
         * if they're all positive then it turns to the right
         */
        rightFront.setTargetPosition(RightFront);
        rightRear.setTargetPosition(RightRear);
        leftFront.setTargetPosition(LeftFront);
        leftRear.setTargetPosition(LeftRear);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(power);

        while(rightFront.isBusy() && rightRear.isBusy() && leftFront.isBusy() && leftRear.isBusy()){

        }
        StopDriving();
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void ArmUp(double power, int distance){
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setTargetPosition(distance);
        rightArm.setTargetPosition(distance);

        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(power);

        while(leftArm.isBusy() && rightArm.isBusy()){

        }
        StopArm();
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void ArmDown(double power, int distance){
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setTargetPosition(-distance);
        rightArm.setTargetPosition(-distance);

        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Drive(power);

        while(leftArm.isBusy() && rightArm.isBusy()){

        }
        StopArm();
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

}


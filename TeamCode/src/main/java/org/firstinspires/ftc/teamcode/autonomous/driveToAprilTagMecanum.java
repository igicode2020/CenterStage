/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "April Tag Driving", group = "--")
// @Disabled
public class driveToAprilTagMecanum extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;

    static public double[] values = new double[3];
    static public int id = 1;

    @Override
    public void runOpMode() {
        //Map motors
        FRM = hardwareMap.get(DcMotorEx.class, "frontRight");
        BRM = hardwareMap.get(DcMotorEx.class, "backRight");
        FLM = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BLM = hardwareMap.get(DcMotorEx.class, "backLeft");

        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        BRM.setDirection(DcMotorEx.Direction.REVERSE);
        FRM.setDirection(DcMotorEx.Direction.REVERSE);

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                updateValues();
                telemetry.addData("x", values[0]);
                telemetry.addData("y", values[1]);
                telemetry.addData("yaw", values[2]);
                telemetry.update();
//            double x = values[0];
//            double y = values[1];
//            double yaw = values[2];
                //correct yaw to align robot parallel to backdrop
//            while (values[2] >= 2 || values[2] <= -2) {
//                //TO DO add acceleration turning
//                if (values[2] > 0) {
//                    FRM.setPower(0.1);
//                    BRM.setPower(0.1);
//                    FLM.setPower(-0.1);
//                    BLM.setPower(-0.1);
//                } else if (values[2] < 0) {
//                    FLM.setPower(0.1);
//                    BLM.setPower(0.1);
//                    FRM.setPower(-0.1);
//                    BRM.setPower(-0.1);
//                }
//                updateValues();
//            }
                FLM.setPower(0);
                BLM.setPower(0);
                FRM.setPower(0);
                BRM.setPower(0);
                while (values[1] >= 11 || values[1] <= 9 || values[0] >= 1 || values[0] <= -1) {
                    //TO DO make sure angles line up with wheel direction
                    double direction = Math.atan(values[0] / values[1]);
                    if (values[1] < 10) {
                        FRM.setPower(0.1);
                        BRM.setPower(0.1);
                        FLM.setPower(0.1);
                        BLM.setPower(0.1);
                    } else if (values[1] > 10) {
                        FRM.setPower(-0.1);
                        BRM.setPower(-0.1);
                        FLM.setPower(-0.1);
                        BLM.setPower(-0.1);
                    }
                    if (values[0] > 0) {
                        FRM.setPower(0.1);
                        BRM.setPower(-0.1);
                        FLM.setPower(-0.1);
                        BLM.setPower(0.1);
                    } else if (values[2] < 0) {
                        FRM.setPower(-0.1);
                        BRM.setPower(0.1);
                        FLM.setPower(0.1);
                        BLM.setPower(-0.1);
                    }
                    updateValues();
                    telemetry.addData("x", values[0]);
                    telemetry.addData("y", values[1]);
                    telemetry.addData("yaw", values[2]);
                    telemetry.addData("direction", direction);
                    telemetry.update();
                }
            FLM.setPower(0);
            BLM.setPower(0);
            FRM.setPower(0);
            BRM.setPower(0);
            }
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        // builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void updateValues() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double x = 0;
        double y = 0;
        double yaw = 0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                //might be able to optimize this by not going through every detection
                if(detection.id == id) {
                    x = detection.ftcPose.x;
                    y = detection.ftcPose.y;
                    yaw = detection.ftcPose.yaw;
                    break;
                }
            }
        }   // end for() loop
        values[0] = x;
        values[1] = y;
        values[2] = yaw;

    }   // end method updateValues()

}   // end class

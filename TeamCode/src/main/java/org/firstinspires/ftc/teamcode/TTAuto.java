/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Normal", group = "Pushbot")
public class TTAuto extends LinearOpMode {


    /* Declare OpMode members. */
    TTHardware robot = new TTHardware();

    private ElapsedTime runtime = new ElapsedTime();// Use a Pushbot's hardware

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_HEX = 288;
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double SPOOL_DIAMETER_INCHES = 2.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_SPOOL = (COUNTS_PER_MOTOR_HEX * DRIVE_GEAR_REDUCTION) /
            (SPOOL_DIAMETER_INCHES * 3.14);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.4;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.35;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable


    BNO055IMU imu;
    Orientation angles;

    private static final String VUFORIA_KEY =
            "AeWr4R//////AAABmbYsM7cHZkA+sRMzs/bFu44j3cYkmMaU6H+xbpPJQ9SynROk2nudJ0sVbgyn7Gc+hGMZ8rFyx0+8iSoQ8O1LZ5+V+4OcX1sL9z1puLugUplscZg7Q53zbPd3BnkON+tt6fFC0VNDNwUEACrMk+TM9szoroXvoJ5PYvywBFuVVq559PhuXwiobyzmMQI7Pb+gXlQj5EmvKn6etHHmWka2xLh1yq85NBuAFaVnlumgvaE+XQywedFTfZVGr6bI0iAvJLci37969ClMcWj1gPRHMq13paRiNoRfRm8HtkJgzP8WTPQIGeNxYSCV2qnppIACTLkxet8jIH6+evJ579rOhvxYxxd0ot7eDAtdfYQqqCPh";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
//    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;
    private WebcamName webcamName = null;

    VuforiaLocalizer vuforia = null;
    OpenCvCamera vuforiaPassthroughCam;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    String pictureName = "No Target Identified";
    String targetDistance = "No Target Identified";

    ArrayList<String> targetArrayList = new ArrayList<>();
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private TFObjectDetector tfod;

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    public int parkingPosition = 0;

    private boolean targetVisible = false;

    OpenCvCamera camera;


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
    AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx,fy, cx, cy);

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        //      while(opModeInInit()) {

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        /*
         * Setup Vuforia
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = "AeWr4R//////AAABmbYsM7cHZkA+sRMzs/bFu44j3cYkmMaU6H+xbpPJQ9SynROk2nudJ0sVbgyn7Gc+hGMZ8rFyx0+8iSoQ8O1LZ5+V+4OcX1sL9z1puLugUplscZg7Q53zbPd3BnkON+tt6fFC0VNDNwUEACrMk+TM9szoroXvoJ5PYvywBFuVVq559PhuXwiobyzmMQI7Pb+gXlQj5EmvKn6etHHmWka2xLh1yq85NBuAFaVnlumgvaE+XQywedFTfZVGr6bI0iAvJLci37969ClMcWj1gPRHMq13paRiNoRfRm8HtkJgzP8WTPQIGeNxYSCV2qnppIACTLkxet8jIH6+evJ579rOhvxYxxd0ot7eDAtdfYQqqCPh";
        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK;
        // Uncomment this line below to use a webcam
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Create a Vuforia passthrough "virtual camera"
        vuforiaPassthroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);


        vuforiaPassthroughCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                vuforiaPassthroughCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                vuforiaPassthroughCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                vuforiaPassthroughCam.setPipeline(aprilTagDetectionPipeline);


                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                vuforiaPassthroughCam.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targets);

        identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);


        final float CAMERA_FORWARD_DISPLACEMENT = 9.5f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        targets.activate();

        robot.init(hardwareMap);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftForwardDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightForwardDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        telemetry.addLine("Init Finished");
        telemetry.update();


//
//        int cameraMonitorViewIdAT = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewIdAT", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewIdAT);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//
//



        waitForStart();

        findCone();

        robot.cameraServo.setPosition(.74); // turn  camera
        robot.pickup.setPosition(0);

        sleep(250);
        moveSlide(1, 38, 1);
        sleep(500);
        ArrayList<String> targetResults = findTarget();
        sleep(1000);

        if (targetResults.isEmpty()) {
            telemetry.addLine("Couldn't Find Photo, Looking to the right");
            telemetry.update();

            robot.cameraServo.setPosition(.3); // Turn right
            sleep(1500);

            targetResults = findTarget();
            sleep(1000);

            if(targetResults.size() != 0) {
                String picture = targetResults.get(0);
                runAuto(picture);

            } else { //if it doesn't see picture
                if(parkingPosition == 0) { //blind

                    gyroDrive(1, 30, 0); // park in 2 if it's blind

                } else { //It doesn't see the picture but sees the cone

                    gyroDrive(1, -24, 0);
                    if(parkingPosition == 1) {
                        gyroStrafe(1, -24, 0);
                        telemetry.addLine("Parked in position 1");
                        telemetry.update();
                    } else if(parkingPosition == 2) {
                        telemetry.addLine("Parked in position 2");
                        telemetry.update();
                    } else if(parkingPosition == 3) {
                        gyroStrafe(1, 24,0);
                        telemetry.addLine("Parking in position 3");
                        telemetry.update();

                    }
                }
            }

        } else { // if it finds something right away
            telemetry.addLine("Found where I am, beginning to stack.");
            String picture = targetResults.get(0);
            telemetry.addLine(targetResults.get(0));
            telemetry.update();
            runAuto(picture);
        }
    }

    public void runAuto(String picture) {
        if (picture == "Red Audience Wall" || picture == "Blue Rear Wall") {
            gyroStrafe(.5, -21, 0); //strafe right
            gyroHold(1, 0, 1);
            gyroDrive(0.01, -23, 0); //negative is forward
            gyroTurn(0.3, -37);
            gyroDrive(0.1, -6, -36); //negative is forward drive to pole
            sleep(1000);

            moveSlide(1, 180, 3);
            robot.slide.setPower(.75);
            sleep(250);
            robot.pickup.setPosition(.22);
            sleep(250);
            robot.slide.setPower(0);


            if(parkingPosition == 1) {
                gyroDrive(.2, 18, -1); //postive is backwards
                gyroTurn(.5, 0);

                telemetry.addLine("Parked in position 1");
            }
            if(parkingPosition == 2) {
                gyroDrive(.5, 18, -1);
                gyroTurn(.5,0);
                gyroStrafe(.2, 27, 0); //strafe left
                telemetry.addLine("Parked in position 2");
            }
            if(parkingPosition == 3) {
                gyroDrive(.5, 18, -1);
                gyroTurn(.5, -1);
                gyroStrafe(.2, 60, 0); //strafe left
                telemetry.addLine("Parked in position 3");
            }


        } else if (picture == "Red Rear Wall" || picture == "Blue Audience Wall") {
            gyroStrafe(.5, 32, 0); // strafe left
            gyroHold(1, 0, 1);
            gyroDrive(0.01, -24, 0); //negative is forward
            gyroTurn(.3, 34);
            gyroDrive(0.1, -8, 35); //negative is forward drive to pole
            sleep(1000);

            moveSlide(1, 180, 3);
            robot.slide.setPower(.75);
            sleep(250);
            gyroHold(1,32, 1);
            robot.pickup.setPosition(.22);
            sleep(250);
            robot.slide.setPower(0);


            //back away from pole and turn to 0
            gyroDrive(.5, 14, 35); //positive is backwards
            gyroTurn(.5, -1);
            gyroHold(.5, -1, 1);

            if(parkingPosition == 1) {

                gyroStrafe(.5, -60, 0);
                telemetry.addLine("Parked in position 1");
            }
            if(parkingPosition == 2) {

                gyroStrafe(.5, -30, 0);
                telemetry.addLine("Parked in position 2");
            }
            if(parkingPosition == 3) {

                telemetry.addLine("Parked in position 3");

            }
            sleep(2000);

        }





    }


    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     newRForwardTarget;
        int     newLForwardTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  leftFSpeed;
        double  rightFSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() - moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;
            newRForwardTarget = robot.rightForwardDrive.getCurrentPosition() - moveCounts;
            newLForwardTarget = robot.leftForwardDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftForwardDrive.setTargetPosition(newLForwardTarget);
            robot.rightForwardDrive.setTargetPosition(newRForwardTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.

            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            robot.leftForwardDrive.setPower(speed);
            robot.rightForwardDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()
                            && robot.leftForwardDrive.isBusy() && robot.rightForwardDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                leftFSpeed = speed - steer;
                rightFSpeed = speed + steer;


                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                    leftFSpeed /= max;
                    rightFSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);
                robot.leftForwardDrive.setPower(leftFSpeed);
                robot.rightForwardDrive.setPower(rightFSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();


            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightForwardDrive.setPower(0);
            robot.leftForwardDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroStrafe(double speed, double distance, double angle) {
        int newLeftTarget;
        int newRightTarget;
        int newRForwardTarget;
        int newLForwardTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double rightFSpeed;
        double leftFSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;
            newRForwardTarget = robot.rightForwardDrive.getCurrentPosition() + moveCounts;
            newLForwardTarget = robot.leftForwardDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftForwardDrive.setTargetPosition(newLForwardTarget);
            robot.rightForwardDrive.setTargetPosition(newRForwardTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                robot.leftDrive.setPower(speed);
                robot.rightDrive.setPower(speed);
                robot.leftForwardDrive.setPower(speed);
                robot.rightForwardDrive.setPower(speed);



            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()
                            && robot.leftForwardDrive.isBusy() && robot.rightForwardDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                leftFSpeed = speed - steer;
                rightFSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                    leftFSpeed /= max;
                    rightFSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);
                robot.leftForwardDrive.setPower(leftFSpeed);
                robot.rightForwardDrive.setPower(rightFSpeed);
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightForwardDrive.setPower(0);
            robot.leftForwardDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftForwardDrive.setPower(0);
        robot.rightForwardDrive.setPower(0);
    }

    public void moveSlide(double speed, double slideInches, double timeoutS) {
        int spoolTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            spoolTarget = robot.slide.getCurrentPosition() + (int)(slideInches * COUNTS_PER_INCH_SPOOL);
            robot.slide.setTargetPosition(spoolTarget);


            // Turn On RUN_TO_POSITION
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.slide.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.slide.isBusy())) {

                // Display it for the driver.
            /*    telemetry.addData("Path1",  "Running to %7d :%7d", spoolTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.spool.getCurrentPosition());
           */
            }

            // Stop all motion;
            robot.slide.setPower(0);
            robot.slide.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }}


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);
        robot.leftForwardDrive.setPower(leftSpeed);
        robot.rightForwardDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);


    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public ArrayList findTarget() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;


                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();


                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }


                VectorF translation = lastLocation.getTranslation();
                if (targetVisible) { // Get the X coords, divide to get inch, divide to get feet.
                    pictureName = trackable.getName();

                }
                targetDistance = "-" + (translation.get(0) / mmPerInch / 12);

                ;
                targetArrayList.add(pictureName);
                targetArrayList.add(targetDistance);

            }


        }

        return targetArrayList;
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

    }

    void findCone() {


        telemetry.setMsTransmissionInterval(50);


        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();


        if(currentDetections.size() != 0) {
            telemetry.addLine("In if");
            telemetry.update();
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                telemetry.update();

                tagToTelemetry(tagOfInterest);
                parkingPosition = tagOfInterest.id;
            } else {
                telemetry.addLine("Don't see tag of interest :(");
                telemetry.update();
            }
        }
    }
}

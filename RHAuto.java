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

@Autonomous(name = "RHAuto", group = "Pushbot")
public class RHAuto extends LinearOpMode {


    /* Declare OpMode members. */
    RHHardware robot = new RHHardware();

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
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;
    private WebcamName webcamName = null;
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

    public int parkingPosition;

    private boolean targetVisible = false;

    @Override
    public void runOpMode() {
        //      while(opModeInInit()) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters camParameters = new VuforiaLocalizer.Parameters();
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        vuforiaParameters.cameraName = webcamName;


        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        vuforiaParameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targets);

        telemetry.addData(">", String.valueOf(allTrackables));
        telemetry.update();


        identifyTarget(0, "Red Audience Wall", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Red Rear Wall", halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Blue Rear Wall", halfField, oneAndHalfTile, mmTargetHeight, 90, 0, -90);


        initTfod();
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }


        // Name and locate each trackable object


        final float CAMERA_FORWARD_DISPLACEMENT = 9.5f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(vuforiaParameters.cameraName, cameraLocationOnRobot);
        }

        robot.init(hardwareMap);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftForwardDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightForwardDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        targets.activate();

        telemetry.addLine("Init Finished");
        telemetry.update();
        waitForStart();


        if (tfod != null) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {


                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

//                    telemetry.addData("", " ");
//                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);



                    if (recognition.getLabel().contains("1")) {
                        parkingPosition = 1;
                        telemetry.addLine("1");
                        telemetry.update();
                    }


                    if (recognition.getLabel().contains("2")) {
                        parkingPosition = 2;
                        telemetry.addLine("2");
                        telemetry.update();
                    }

                    if (recognition.getLabel().contains("3")) {
                        parkingPosition = 3;
                        telemetry.addLine("3");
                        telemetry.update();
                    }
                }
                telemetry.update();
            }
        }
        robot.cameraServo.setPosition(.7); // turn  camera left
        sleep(1000);
        ArrayList<String> targetResults = findTarget();
        sleep(1000);
        if (targetResults.isEmpty()) {
            telemetry.addLine("Couldn't Find Photo, Looking to the right");
            telemetry.update();


            robot.cameraServo.setPosition(.3); // Turn right
            sleep(2000);
            targetResults = findTarget();
            if(!
                    targetResults.get(0).isEmpty()) {

                runAuto(targetResults.get(0));
            }
            if (targetResults.isEmpty()) {
                telemetry.addLine("Im lost. Parking now");
                telemetry.update();
                sleep(10000);

            }
        } else { // if it finds something right away
            telemetry.addLine("Found where I am, beginning to stack.");
            String picture = targetResults.get(0);
            telemetry.addLine(targetResults.get(0));
            telemetry.update();



            if (picture == "Red Audience Wall") {
                gyroDrive(.5, 20, 0);
                gyroHold(1, 0, 1);


                if(parkingPosition == 1) {
                    gyroStrafe(1, -20, 0);

                    telemetry.addLine("Parked in position 1");
                }
                if(parkingPosition == 2) {
                    telemetry.addLine("Parked in position 2");
                }
                if(parkingPosition == 3) {
                    gyroStrafe(1, 20, 0);
                    telemetry.addLine("Parked in position 3");
                }


                //stack
            } else if (picture == "Red Rear Wall") {
                gyroDrive(.5, 20, 0);
                gyroHold(1, 0, 1);


                if(parkingPosition == 1) {
                    gyroStrafe(1, -20, 0);

                    telemetry.addLine("Parked in position 1");
                }
                if(parkingPosition == 2) {
                    telemetry.addLine("Parked in position 2");
                }
                if(parkingPosition == 3) {
                    gyroStrafe(1, 20, 0);
                    telemetry.addLine("Parked in position 3");
                }
            } else if (picture == "Blue Rear Wall") {
                gyroDrive(.5, 20, 0);
                gyroHold(1, 0, 1);


                if(parkingPosition == 1) {
                    gyroStrafe(1, -20, 0);

                    telemetry.addLine("Parked in position 1");
                }
                if(parkingPosition == 2) {
                    telemetry.addLine("Parked in position 2");
                }
                if(parkingPosition == 3) {
                    gyroStrafe(1, 20, 0);
                    telemetry.addLine("Parked in position 3");
                }
            } else if (picture == "Blue Audience Wall") {
                gyroDrive(.5, 20, 0);
                gyroHold(1, 0, 1);


                if(parkingPosition == 1) {
                    gyroStrafe(1, -20, 0);

                    telemetry.addLine("Parked in position 1");
                }
                if(parkingPosition == 2) {
                    telemetry.addLine("Parked in position 2");
                }
                if(parkingPosition == 3) {
                    gyroStrafe(1, 20, 0);
                    telemetry.addLine("Parked in position 3");
                }
            }


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
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() - moveCounts;
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

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    public void runAuto(String picture) {
        if (picture == "Red Audience Wall") {
            gyroDrive(.5, 40, 0);
            gyroHold(1, 0, 1);


            if(parkingPosition == 1) {
                gyroStrafe(1, -20, 0);

                telemetry.addLine("Parked in position 1");
            }
            if(parkingPosition == 2) {
                telemetry.addLine("Parked in position 2");
            }
            if(parkingPosition == 3) {
                gyroStrafe(1, 20, 0);
                telemetry.addLine("Parked in position 3");
            }


            //stack
        } else if (picture == "Red Rear Wall") {
            gyroDrive(.5, 40, 0);
            gyroHold(1, 0, 1);


            if(parkingPosition == 1) {
                gyroStrafe(1, -20, 0);

                telemetry.addLine("Parked in position 1");
            }
            if(parkingPosition == 2) {
                telemetry.addLine("Parked in position 2");
            }
            if(parkingPosition == 3) {
                gyroStrafe(1, 20, 0);
                telemetry.addLine("Parked in position 3");
            }
        } else if (picture == "Blue Rear Wall") {
            gyroDrive(.5, 40, 0);
            gyroHold(1, 0, 1);


            if(parkingPosition == 1) {
                gyroStrafe(1, -0, 0);

                telemetry.addLine("Parked in position 1");
            }
            if(parkingPosition == 2) {
                telemetry.addLine("Parked in position 2");
            }
            if(parkingPosition == 3) {
                gyroStrafe(1, 20, 0);
                telemetry.addLine("Parked in position 3");
            }
        } else if (picture == "Blue Audience Wall") {
            gyroDrive(.5, 40, 0);
            gyroHold(1, 0, 1);


            if(parkingPosition == 1) {
                gyroStrafe(1, -20, 0);

                telemetry.addLine("Parked in position 1");
            }
            if(parkingPosition == 2) {
                telemetry.addLine("Parked in position 2");
            }
            if(parkingPosition == 3) {
                gyroStrafe(1, 20, 0);
                telemetry.addLine("Parked in position 3");
            }
        }


    }
    }


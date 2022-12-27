/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode Sample illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear opmode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
*/

@TeleOp(name="RHOpMode", group="Robot")
public class RHOpMode extends LinearOpMode {
    private boolean sean = false;
    public HardwareMap tthw = null;
    RHHardware robot = new RHHardware();

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.


    @Override
    public void runOpMode() {




        // initialize all the hardware, using the hardware class. See how clean and simple this is?




        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.init(hardwareMap);
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).

            double arm = gamepad2.left_stick_y;
            boolean clawOpen = gamepad2.left_bumper;
            boolean clawClose = gamepad2.right_bumper;

            double drive = -gamepad1.left_stick_x;
            double strafe = gamepad1.left_stick_y;
            double twist = 0;
            double normalTurn= gamepad1.right_stick_x;
            double cameraServo = gamepad2.left_trigger;
            double armServo = gamepad2.right_stick_y;

            double leftPower;
            double rightPower;
            double leftForwardPower;
            double rightForwardPower;



            if(clawOpen) {
                robot.claw.setPosition(45);
            }
            if(clawClose) {
                robot.claw.setPosition(-45);
            }











            //robot.cameraServo.setPosition(0.0);
            /*
             * If we had a gyro and wanted to do field-oriented control, here
             * is where we would implement it.
             *
             * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
             * coordinate (strafe, drive), and we just rotate it by the gyro
             * reading minus the offset that we read in the init() method.
             * Some rough pseudocode demonstrating:
             *
             * if Field Oriented Control:
             *     get gyro heading
             *     subtract initial offset from heading
             *     convert heading to radians (if necessary)
             *     new strafe = strafe * cos(heading) - drive * sin(heading)
             *     new drive  = strafe * sin(heading) + drive * cos(heading)
             *
             * If you want more understanding on where these rotation formulas come
             * from, refer to
             * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
             */

            // You may need to multiply some of these by -1 to invert direction of
            // the motor.  This is not an issue with the calculations themselves.
            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)

            };

            // Because we are adding vectors and motors only take values between
            // [-1,1] we may need to normalize them.

            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.
            double max = Math.abs(speeds[0]);
            for (int i = 0; i < speeds.length; i++) {
                if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
            }

            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }


            // apply the calculated values to the motors.
            if(normalTurn == 0) {
                telemetry.addData(">", "Strafe");
                telemetry.update();
                robot.leftForwardDrive.setPower(speeds[0]);
                robot.rightForwardDrive.setPower(speeds[1]);
                robot.leftDrive.setPower(speeds[2]);
                robot.rightDrive.setPower(speeds[3]);
            }
            else {
                telemetry.addData(">","turn");
                telemetry.update();
                leftPower = Range.clip(normalTurn, -1.0, 1.0);
                rightPower = Range.clip(normalTurn, -1.0, 1.0);
                rightForwardPower = Range.clip(normalTurn, -1.0, 1.0);
                leftForwardPower = Range.clip(normalTurn, -1.0, 1.0);


                robot.leftForwardDrive.setPower(leftForwardPower);
                robot.rightForwardDrive.setPower(rightForwardPower);
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }


            if(gamepad1.dpad_down)
            {
                sean = true;
            }
            if(gamepad1.dpad_up)
            {
                sean = false;
            }

            if(sean) {
                robot.leftDrive.setPower(robot.leftDrive.getPower() * 0.2);
                robot.rightDrive.setPower(robot.rightDrive.getPower() * 0.2);
                robot.leftForwardDrive.setPower(robot.leftForwardDrive.getPower() * 0.2);
                robot.rightForwardDrive.setPower(robot.rightForwardDrive.getPower() * 0.2);
                if(gamepad1.right_stick_x > 0) {
                    leftPower = Range.clip(drive + twist, -0.5, 0.5) ;
                    rightPower = Range.clip(drive - twist, -0.5, 0.5) ;
                    rightForwardPower = Range.clip(drive - twist, -0.5, 0.5) ;
                    leftForwardPower = Range.clip(drive + twist, -0.5, 0.5) ;


                }

            }

            robot.arm.setPower(Range.clip(arm, -1, 1));


            robot.armServo.setPower(Range.clip(armServo, -1, 1));


        }

    }
}

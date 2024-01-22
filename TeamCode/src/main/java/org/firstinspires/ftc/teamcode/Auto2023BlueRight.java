// /* Copyright (c) 2017 FIRST. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted (subject to the limitations in the disclaimer below) provided that
//  * the following conditions are met:
//  *
//  * Redistributions of source code must retain the above copyright notice, this list
//  * of conditions and the following disclaimer.
//  *
//  * Redistributions in binary form must reproduce the above copyright notice, this
//  * list of conditions and the following disclaimer in the documentation and/or
//  * other materials provided with the distribution.
//  *
//  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//  * promote products derived from this software without specific prior written permission.
//  *
//  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */
// 
// package org.firstinspires.ftc.teamcode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// 
// /**
//  * This file illustrates the concept of driving a path based on time.
//  * The code is structured as a LinearOpMode
//  *
//  * The code assumes that you do NOT have encoders on the wheels,
//  *   otherwise you would use: RobotAutoDriveByEncoder;
//  *
//  *   The desired path in this example is:
//  *   - Drive forward for 3 seconds
//  *   - Spin right for 1.3 seconds
//  *   - Drive Backward for 1 Second
//  *
//  *  The code is written in a simple form with no optimizations.
//  *  However, there are several ways that this type of sequence could be streamlined,
//  *
//  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
//  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
//  */
// 
// @Autonomous(name="don't use this 2023 br", group="Robot")
// 
// public class Auto2023BlueRight extends LinearOpMode{
//     private DcMotor fL   = null;
//     private DcMotor bL = null;
//     private DcMotor fR   = null;
//     private DcMotor bR = null;
//     private DcMotor arm = null;
//     private Servo sL = null;
//     private Servo sR = null;
//     private ElapsedTime     runtime = new ElapsedTime();
// 
// 
//     @Override
//     public void runOpMode() {
// 
//         // Initialize the drive system variables.
//         fL  = hardwareMap.get(DcMotor.class, "fL");
//         fR = hardwareMap.get(DcMotor.class, "fR");
//         bL  = hardwareMap.get(DcMotor.class, "bL");
//         bR = hardwareMap.get(DcMotor.class, "bR");
//         arm = hardwareMap.get(DcMotor.class, "arm");
//         sR = hardwareMap.get(Servo.class, "sR");
//         sL = hardwareMap.get(Servo.class, "sL");
//         
//         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//         // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//         fL.setDirection(DcMotor.Direction.REVERSE);
//         bL.setDirection(DcMotor.Direction.REVERSE);
//         fR.setDirection(DcMotor.Direction.REVERSE);
//         bR.setDirection(DcMotor.Direction.REVERSE);
//         arm.setDirection(DcMotor.Direction.REVERSE);
// 
//         // Send telemetry message to signify robot waiting;
//         telemetry.addData("Status", "Ready to run");    //
//         telemetry.update();
// 
//         // Wait for the game to start (driver presses PLAY)
//         waitForStart();
//         // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
//         // Step 1:  Drive forward for 3 seconds and some other, less important stuff too
//         grab();
//         strafeDirection(0.4, 900);
//         sleep(1000);
//         arm(0.8, 1100);
//         sleep(1000);
//         moveDirection(0.6, 0.6, 740);
//         sleep(1000);
//         precDirection(0.3, -0.3, 0.3, -0.3, 0.1, 200);
//         sleep(1000);
//         release();
//         sleep(1000);
//         precDirection(-0.3, 0.3, -0.3, 0.3, 0.1, 200);
//         sleep(1000);
//         moveDirection(-0.6, -0.6, 750);
//         sleep(1000);
//         strafeDirection(0.5, 1000);
//         sleep(1000);
//         
//         
//         
//         
//         // Step 2: done
//     
// 
//     
// 
//         
//         // Step 4:  Stop
//         fL.setPower(0);
//         fR.setPower(0);
//         bL.setPower(0);
//         bR.setPower(0);
//         arm.setPower(0);
//         sleep(2000);
// 
//         telemetry.addData("Path", "Complete", "The thing has been done");
//         telemetry.update();
//         sleep(1000);
//     }
//     private void moveDirection(double lP, double rP,  double s){
//         runtime.reset();
//         while (opModeIsActive() && (runtime.milliseconds() < s)) {
//             fL.setPower(lP);
//             fR.setPower(rP);
//             bL.setPower(lP);
//             bR.setPower(rP);
//         }
//         fL.setPower(0);
//         fR.setPower(0);
//         bL.setPower(0);
//         bR.setPower(0);
//     }
//     private void strafeDirection(double p, double s){
//         runtime.reset();
//         while (opModeIsActive() && (runtime.milliseconds() < s)) {
//             fL.setPower(p);
//             fR.setPower(-p);
//             bL.setPower(-p);
//             bR.setPower(p);
//             telemetry.addData("mili", runtime.milliseconds());
//             telemetry.addData("s", s);
//             telemetry.update();
//         }
//         fL.setPower(0);
//         fR.setPower(0);
//         bL.setPower(0);
//         bR.setPower(0);
//     }
//     private void precDirection(double lF, double rF, double lB, double rB,  double a, double s){
//         runtime.reset();
//         while(opModeIsActive() && (runtime.milliseconds() < s)) {
//             fL.setPower(lF);
//             fR.setPower(rF);
//             bL.setPower(lB);
//             bR.setPower(rB);
//             arm.setPower(a);
//         }
//         fL.setPower(0);
//         fR.setPower(0);
//         bL.setPower(0);
//         bR.setPower(0);
//         arm.setPower(0.1);
//     }
// 
//     private void arm(double aP, double s){
//         runtime.reset();
//         while(opModeIsActive()&& (runtime.milliseconds() < s)) {
//             arm.setPower(aP);
//         }
//         arm.setPower(0.05);
//     }
//     private void grab(){
//         sL.setPosition(0.6);
//         sR.setPosition(0.8);
//         
//     }
//     private void release(){
//         sL.setPosition(0.75);
//         sR.setPosition(0.7);
//     }
//     
//     
// }
// 
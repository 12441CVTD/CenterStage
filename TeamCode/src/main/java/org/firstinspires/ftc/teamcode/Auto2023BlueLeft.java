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
// import com.vuforia.PIXEL_FORMAT;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
// import org.firstinspires.ftc.robotcore.external.ClassFactory;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
// import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
// import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.vuforia.Image;
// import com.vuforia.Vuforia;
// import java.nio.ByteBuffer;
// 
// import java.util.List;
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
// @Autonomous(name="SEASON 2023 LEEEEEEEFFFTTT SIDE CAMERA", group="Robot")
// 
// public class Auto2023BlueLeft extends LinearOpMode{
// 
//     /* Declare OpMode members. */
//     private Camera cam;
//     private CameraManager CM;
//     private DcMotor fL   = null;
//     private DcMotor bL = null;
//     private DcMotor fR   = null;
//     private DcMotor bR = null;
//     private DcMotor arm = null;
//     private Servo sL = null;
//     private Servo sR = null;
//     private Servo Turny = null;
//     private ElapsedTime     runtime = new ElapsedTime();
//     private VuforiaLocalizer vuforia;
//     private VuforiaLocalizer.CloseableFrame CF;
//     private int park;
//     private final String VUFORIA_KEY = "AR2scin/////AAABmYgpWNfrBkOVpIfVuIW8A6RovbwHCxkhdIUdnl/WoKKmqbjhIJ8A/em8d9xhswlf0J/1BfNo6uEK1E6ILXDAjsplJovHZCAKlbNKCb1jb3ZyiyRwgikHJYC6nF5QLEibm/cFB/SFB1qDjg1TF9RfhahrKd/EDhVk6t4oFaPKTWzLTYE33nW3/p40mvUHq8s2JpZSAm0a6yyaDrCfMj4luhJGXREJx3HqwGWE7OsjsgGdINdXqkeJVRGFj+eDJTqGDrT/FYb4S644qyLSeSi+yYir7NRq7Afkhie4LAd2RRwea5P539EP7QfzTrFKV8ioavO/czxn2TrGaNP072OtrSG/ik3VsH3601mBAoaheM2A";
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
//         Turny = hardwareMap.get(Servo.class, "Turny");
//         // WebcamName Webcam = hardwareMap.get(WebcamName.class,"Webcam 1");
//         // VuforiaLocalizer.Parameters Vparameters = new VuforiaLocalizer.Parameters();
//         // Vparameters.cameraName = Webcam;
//         
//         // vuforia = ClassFactory.getInstance().createVuforia(Vparameters);
//         // cam = vuforia.getCamera();
//         // vuforia.setFrameQueueCapacity(2);
//         //sleep(200);
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
//         fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 
//         initVuforia();
//         
//         // Wait for the game to start (driver presses PLAY)
//         waitForStart();
//         // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
//         // Step 1:  Drive forward for 3 seconds and some other, less important stuff too'
//         
//         
//         Turny.setPosition(0.235);
//         grab();
//         park = getCameraReading();
//         if(park==1)
//         telemetry.addData("1st area", "ok");
//         if(park==2)
//         telemetry.addData("2nd area", "ok");
//         if(park==3)
//         telemetry.addData("3rd area", "ok");
//         telemetry.update();
//         sleep(200);
//         moveDirection(0.4, 0.4, 120);
//         sleep(150);
//         strafeDirection(-0.7, 500);
//         sleep(150);
//         arm(0.9, 800);
//         sleep(150);
//         moveDirection(0.6, 0.6, 500);
//         sleep(500);
//         moveDirection(0.4, 0.4, 240);
//         sleep(250);
//         release();
//         sleep(400);
//         moveDirection(-0.4, -0.4, 240);
//         sleep(250);
//         strafeDirection(0.7, 575);
//         sleep(150);
//         strafeDirection(-0.7, 190); 
//         sleep(150);
//         //moveDirection(-0.3, 0.3, 100);
//         //sleep(150);
//         //moveDirection(0.7, 0.7, 530);
//         /*moveDirection(0.7, 0.7, 700);
//         sleep(100);
//         strafeDirection(-0.5, 150);
//         sleep(100);
//         moveDirection(-0.7, 0.7, 540);
//         sleep(100);
//         strafeDirection(0.7, 290);
//         arm(0.8, 630);
//         sleep(100);
//         moveDirection(0.4, 0.4, 200);
//         sleep(100);
//         //release();
//         sleep(100);
//         moveDirection(-0.4, -0.4, 125);
//         sleep(100);
//         arm(-0.8, 630);
//         sleep(100);
//         strafeDirection(0.7, 600);
//         precDirection(0.6, 0.0, 0.0, 0.6, 0.05, 500);
//         sleep(100);
//         moveDirection(-0.82, 0.82, 780);
//         */sleep(100);
// 
// 
// 
// 
//         
//         //moveDirection(0.55, 0.55, 1240);
//         sleep(1000);
//         if(park==1){
//             strafeDirection(-0.55, 1240);
//         }
//         else if(park==2){
//             sleep(100);
//         }
//         else if(park==3){
//             strafeDirection(0.55, 1240);
//         }else{
//             
//         }
//         //sleep(5000);
//        
//         
//         
//         // Step 2: done
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
//         
// 
//         telemetry.addData("Path", "Complete", "The thing has been done");
//         telemetry.update();
//         sleep(1000);
//     }
//     
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
//     
//     private void strafeDirection(double p, double s){
//         runtime.reset();
//         while (opModeIsActive() && (runtime.milliseconds() < s)) {
//             fL.setPower(0.9*p);
//             fR.setPower(0.9*-p);
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
//         sL.setPosition(0.45);
//         sR.setPosition(0.48);
//         
//     }
//     private void release(){
//         sL.setPosition(0.61);
//         sR.setPosition(0.32);
//     }
//     private int getCameraReading() {
//         VuforiaLocalizer.CloseableFrame frame = null;
//         try {
//             frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
//         } catch(Exception e) {
//             e.printStackTrace();
//             telemetry.addData("Might be working", "Please");
//             telemetry.update();
//         }
//         if (frame == null) {
//             telemetry.addData("SOMETHING IS ON FIRE", "Contact the fire department");
//             telemetry.update();
//             return 4;
//         }
//         long numImages = frame.getNumImages();
//         Image image = null;
//         for (int i = 0; i < numImages; i++) {
//             if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                 image = frame.getImage(i);
//             }
//         }
//         int[] colors = {0, 0, 0, 0};
//         if (image != null) {
//             ByteBuffer pixels = image.getPixels();
//             byte[] pixelArray = new byte[pixels.remaining()];
//             pixels.get(pixelArray, 0, pixelArray.length);
//             int imgWidth = image.getWidth();
//             int imgHeight = image.getHeight();
//             int[] startingIndexes = getRowStartingIndexes(imgHeight, imgWidth, 60);
//             for (int i = 60 * 5/ 12; i < 60 * 7 / 12; i++) {
//                 for (int j = startingIndexes[i] + imgWidth * 2 / 3; j < startingIndexes[i] + imgWidth * 2 * 2/3; j += 2) {
//                     colors[getColor(pixelArray[j], pixelArray[j+1])]++;
//                     telemetry.addData("width", imgWidth);
//                     telemetry.addData("height", imgHeight);
//                     telemetry.addData("startingIndexes[i]", startingIndexes[i]);
//                     telemetry.addData("yellow", colors[1]);
//                     telemetry.addData("green", colors[0]);
//                     telemetry.addData("purpur", colors[2]);
//                     //telemetry.update();
//                     // sleep(30000);
//                 }
//             }
//                         // sleep(5000);
//             // telemetry.addData("b1", pixelArray[0]);
//             // telemetry.addData("b2", pixelArray[1]);
//             telemetry.addData("yellow", colors[1]);
//             telemetry.addData("green", colors[0]);
//             telemetry.addData("purpur", colors[2]);
//             telemetry.update();
//             for (int i = 0; i < startingIndexes.length; i++)
//                 telemetry.addData("startingIndexes[i]", startingIndexes[i]);
//             telemetry.update();
//             // sleep(30000);
//         }
// 
//         frame.close();
//         int max_index = 0;
//         for (int i = 1; i < 3; i++) {
//             if (colors[i] > colors[max_index])
//                 max_index = i;
//         }
//         
//         telemetry.addData("yellow", colors[1]);
//         telemetry.addData("green", colors[0]);
//         telemetry.addData("purpur", colors[2]);
//         telemetry.update();
//         // sleep(5000);
// 
//         if (max_index == 0)
//             return 1;
//         if (max_index == 1)
//             return 2;
//         else
//             return 3;
//  
//             
//     }
//     private int[] getRowStartingIndexes(int height, int width, int numRows) {
//         int[] newArr = new int[numRows];
//         int stepSize = 2 * height / numRows * width;
//         for (int i = 1; i < numRows; i++) {
//             newArr[i] = i * stepSize;
//         }
//         return newArr;
//     }
//     private int getColor(byte b1, byte b2) {
//         // GGGBBBBB RRRRRGGG;
//         String s1 = String.format("%8s", Integer.toBinaryString(b2 & 0xFF)).replace(' ', '0');
//         String s2 = String.format("%8s", Integer.toBinaryString(b1 & 0xFF)).replace(' ', '0');
//         // RRRRRGGG GGGBBBBB;
//         int[] color = new int[3];
//         String r = s1.substring(0, 5);
//         String g = s1.substring(5) + s2.substring(0, 3);
//         String b = s2.substring(3);
//         color[0] = convertBitStringToInt(r);
//         color[1] = convertBitStringToInt(g);
//         color[2] = convertBitStringToInt(b);
//         double[] hsv = convertRGBtoHSV(color);
//         telemetry.addData("hsv", hsv[0]);
//         telemetry.addData("hsv", hsv[1]);
//         telemetry.addData("hsv", hsv[2]);
//         telemetry.addData("b1", b1);
//         telemetry.addData("b2", b2);
//         telemetry.addData("hsv[2]", hsv[2]);
//         if (hsv[0] >= 225 && hsv[0] <= 315 && hsv[1] > 0.15 && hsv[2] > 0.15)
//             return 2;
//         if (hsv[0] >= 70 && hsv[0] <= 155 && hsv[1] > 0.15 && hsv[2] > 0.2)
//             return 0;
//         if (hsv[0] >= 45 && hsv[0] <= 70 && hsv[1] > 0.15 && hsv[2] > 0.5)
//             return 1;
//         return 3;
//     }
//     private double[] convertRGBtoHSV(int[] rgb) {
//         double rPrime = (double) rgb[0]/31;
//         double gPrime = (double) rgb[1]/63;
//         double bPrime = (double) rgb[2]/31;
//         double cMax = Math.max(rPrime, Math.max(gPrime, bPrime));
//         double cMin = Math.min(rPrime, Math.min(gPrime, bPrime));
//         double delta = cMax - cMin;
//         double[] hsv = new double[3];
// 
//         // calculate hue
//         if (delta == 0)
//             hsv[0] = 0;
//         else if (cMax == rPrime) {
//             double temp = ((gPrime - bPrime) / delta) % 6;
//             if (temp < 0)
//                 temp += 6;
//             hsv[0] = 60 * temp;
//         }
//         else if (cMax == gPrime)
//             hsv[0] = 60 * (((bPrime - rPrime) / delta) + 2);
//         else
//             hsv[0] = 60 * (((rPrime - gPrime) / delta) + 4);
// 
//         // calculate saturation
//         if (cMax == 0)
//             hsv[1] = 0;
//         else
//             hsv[1] = delta / cMax;
// 
//         // calculate value
//         hsv[2] = cMax;
// 
//         return hsv;
//     }
// 
//     private int convertBitStringToInt(String s) {
//         int sum = 0;
//         // Little Endian
//         // int digit = 0;
//         // for (char c : s.toCharArray()) {
//         //     if (c == '1') {
//         //         sum += Math.pow(2, digit);
//         //     }
//         //     digit++;
//         // }
//         // Big Endian
//         int digit = s.length() - 1;
//         for (char c : s.toCharArray()) {
//             if (c == '1') {
//                 sum += Math.pow(2, digit);
//             }
//             digit--;
//         }
//         return sum;
//     }
//     private void initVuforia() {
//         /*
//          * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//          */
//         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
// 
//         parameters.vuforiaLicenseKey = VUFORIA_KEY;
//         parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
// 
//         //  Instantiate the Vuforia engine
//         vuforia = ClassFactory.getInstance().createVuforia(parameters);
//         vuforia.setFrameQueueCapacity(10);
//         // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//         Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//         telemetry.addData("vuforia", "initiated");
//         telemetry.update();
//     }
// }
// 
// 
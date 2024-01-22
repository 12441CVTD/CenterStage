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
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUD
//  
//  
//  
//  
//  
//  
//  ING, BUT NOT LIMITED TO,
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
// import org.firstinspires.ftc.robotcore.internal.system.Deadline;
// import java.util.logging.Logger;
// import java.util.logging.Logger;
// import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
// import org.firstinspires.ftc.robotcore.external.function.Continuation;
// import java.util.concurrent.TimeUnit;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.ClassFactory;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import android.graphics.Bitmap;
// import android.graphics.Color;
// /**
//  * This file illustrates the concept of driving a path based on time.
//  * It uses the common Pushbot hardware class to define the drive on the robot.
//  * The code is structured as a LinearOpMode
//  *
//  * The code assumes that you do NOT have encoders on the wheels,
//  *   otherwise you would use: PushbotAutoDriveByEncoder;
//  *
//  *   The desired path in this example is:
//  *   - Drive forward for 3 seconds
//  *   - Spin right for 1.3 seconds
//  *   - Drive Backwards for 1 Second
//  *   - Stop and close the claw.
//  *
//  *  The code is written in a simple form with no optimizations.
//  *  However, there are several ways that this type of sequence could be streamlined,
//  *
//  * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
//  * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
//  */
// 
// @Autonomous(name="AutoRedLeftDUCK", group="Pushbot")
// 
// public class AutoRedLeftDUCK extends LinearOpMode {
// 
//     /* Declare OpMode members. */
//     
//     //Camera Stuffs - Dont touch
//     private Camera cam;
//     private CameraManager CM;
//     private ElapsedTime runtime = new ElapsedTime();
//     private DcMotor fL = null;
//     private DcMotor bL = null;
//     private DcMotor fR = null;
//     private DcMotor bR = null;
//     private DcMotor motorA = null;
//     private DcMotor motorCarLeft = null;
//     private DcMotor motorCarRight = null;
//     private Servo gripL = null;
//     private Servo gripR = null;
//     private int initPos=0;
//     private int duckSide = -1;
//     private DcMotor[] wheels;
//     private int armHeight;
//     static final double     FORWARD_SPEED = 0.6;
//     static final double     TURN_SPEED    = 0.5;
//     double aLowPower, aHoldPower, aHighPower;
//     private VuforiaLocalizer vuforia;
//     private VuforiaLocalizer.CloseableFrame CF;
//     private android.graphics.Bitmap BM;
//     private java.util.concurrent.BlockingQueue<VuforiaLocalizer.CloseableFrame>  FQ;
//     // This is setting up the gyrosensor
//     BNO055IMU imu;
//     Orientation angles;
// 
//     @Override
//     public void runOpMode() {
//         
//     //again setting up the imu
//     
//     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//     parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//         
//          
//         /*
//          * Initialize the drive system variables.
//          * The init() method of the hardware class does all the work here
//          */
//         fL = hardwareMap.get(DcMotor.class, "fL");
//         bL = hardwareMap.get(DcMotor.class, "bL");
//         fR = hardwareMap.get(DcMotor.class, "fR");
//         bR = hardwareMap.get(DcMotor.class, "bR");
//         wheels = new DcMotor[]{fL,fR,bL,bR};
//         motorA = hardwareMap.get(DcMotor.class, "motorA");
//         motorCarLeft = hardwareMap.get(DcMotor.class, "motorCarLeft");
//         motorCarRight = hardwareMap.get(DcMotor.class, "motorCarRight");
//         gripL = hardwareMap.get(Servo.class, "gripL");
//         gripR = hardwareMap.get(Servo.class, "gripR");
//         imu = hardwareMap.get(BNO055IMU.class, "imu");
//         //cam = hardwareMap.get(WebcameName.class, "Webcam");
//         imu.initialize(parameters);
//         
//         CM=ClassFactory.getInstance().getCameraManager();
//         //CSC = ClassFactory.getInstance().
//         WebcamName WN = hardwareMap.get(WebcamName.class,"Webcam");
//         VuforiaLocalizer.Parameters Vparameters = new VuforiaLocalizer.Parameters();
//         String key = "AR2scin/////AAABmYgpWNfrBkOVpIfVuIW8A6RovbwHCxkhdIUdnl/WoKKmqbjhIJ8A/em8d9xhswlf0J/1BfNo6uEK1E6ILXDAjsplJovHZCAKlbNKCb1jb3ZyiyRwgikHJYC6nF5QLEibm/cFB/SFB1qDjg1TF9RfhahrKd/EDhVk6t4oFaPKTWzLTYE33nW3/p40mvUHq8s2JpZSAm0a6yyaDrCfMj4luhJGXREJx3HqwGWE7OsjsgGdINdXqkeJVRGFj+eDJTqGDrT/FYb4S644qyLSeSi+yYir7NRq7Afkhie4LAd2RRwea5P539EP7QfzTrFKV8ioavO/czxn2TrGaNP072OtrSG/ik3VsH3601mBAoaheM2A";
//         Vparameters.vuforiaLicenseKey = key;
//         Vparameters.cameraName = WN;
//         
//         //  Instantiate the Vuforia engine
//         vuforia = ClassFactory.getInstance().createVuforia(Vparameters);
//         cam = vuforia.getCamera();
//         vuforia.enableConvertFrameToBitmap();
//         vuforia.setFrameQueueCapacity(2);
//         sleep(200);
//           FQ = vuforia.getFrameQueue();
//         
//         //sleep(200);
//         checkPix();
//         
//         
//         
//         //reverse motors
//         fL.setDirection(DcMotor.Direction.REVERSE);
//         bL.setDirection(DcMotor.Direction.REVERSE);
//         fR.setDirection(DcMotor.Direction.FORWARD);
//         bR.setDirection(DcMotor.Direction.FORWARD);
// 
// 
// 
// 
//         initPos = motorA.getCurrentPosition();
//         // Send telemetry message to signify robot waiting;
//         telemetry.addData("Status", "Ready to run");    //
//         telemetry.update();
//         
//         // Wait for the game to start (driver presses PLAY)
//         waitForStart();
// //MAIN LOOP STARTS HERE
//         
//         checkPix();
//         //We made a double so we can change the power later
//         double Power = 0.5;
//         runtime.reset();
//         int low = -35; //change these, the only correct one is the high setting 
//         int mid = -140; //numbers seem correct, arm seems to set to wrong degree
//         int high = -225; //changed from 60 and 100 to 18 lower for bottom 2. arm sets 15-20 too high. dont know how to fix, we should do it later
//         
//         aLowPower = -.1;
//         aHoldPower = armHeight == high?-.4:-.3;
//         aHighPower= -.6;
//         //  
//         
//         armHeight = duckSide==0? low : (duckSide==1? mid : high );
//         ClawSetClosed(true);
//         
//        /* while(runtime.seconds()<30){
//             telemetry.addLine(((Integer)motorA.getCurrentPosition()).toString() );
//             telemetry.update();
//         }*/
//         
//         while(motorA.getCurrentPosition() > armHeight){
//             motorA.setPower(armHeight == low?aHighPower:aHighPower-.05);
//         }
//         telemetry.addLine(((Integer)motorA.getCurrentPosition()).toString());
//         telemetry.update();
//         motorA.setPower(aHoldPower);
//         sleep(2000);
//         
//         /*
//         if(motorA.getCurrentPosition()<armHeight-30){
//             while(motorA.getCurrentPosition() <armHeight){
//             motorA.setPower(armHeight == low?-.05:.1);
//             }
//         }
//         */
//         DriveForSeconds(.25,.2,true);
//         StrafeForSeconds(-1,.6,true);
//         //StrafeForSeconds(1,.01,true);
//         DriveForSeconds(-.35,.5,true);
//         DriveForSeconds(.5,.35,true);
//         telemetry.addLine(String.valueOf(motorA.getCurrentPosition()));
//         telemetry.update();
//         ClawSetClosed(false);
//         DriveForSeconds(-.25,1,true);
//         DriveForSeconds(-.1,1,true);
//         StrafeForSecondsBackwards(-1,.95,true);
//         DriveForSeconds(.45,.3,true);
//         motorA.setPower(.3);
//         sleep(2000);
//         
//         
//        // }
//         
//         
//            
//             telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//             telemetry.update();
// 
//         fL.setPower(0);
//         bL.setPower(0);
//         fR.setPower(0);
//         bR.setPower(0);
//         motorCarLeft.setPower(0);
//         motorCarRight.setPower(0);
// 
// 
//         telemetry.addData("Path", "Complete");
//         telemetry.update();
//         sleep(1000);
//         
//         
//     }
// //MAIN LOOP ENDED
// //CUSTOM METHODS START HERE
//   public void getHeading(){
//             angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//             //This is the angle perpendicular to the ground
//             telemetry.addData("Heading", angles.firstAngle);
//             telemetry.update();
//         }  
//         
//     public void StrafeForSeconds(double p, double secs, boolean armUp){
//         runtime.reset();
//         while(runtime.seconds()<secs){
//             //fP balances backheavy weight distro
//             double fP = p;
//             double bP = p;//*.93;
//             for(int i =0;i<2;i++){
//                 wheels[i].setPower((i>0&&i<3)?fP:-fP);
//             }
//             for(int i =2;i<4;i++){
//                 wheels[i].setPower((i>0&&i<3)?bP:-bP);
//             }
//             if(armUp)CheckSetArmHeight(armHeight,aLowPower,aHoldPower,aHighPower);
//             telemetry.addLine(((Integer)motorA.getCurrentPosition()).toString());
//             telemetry.update();
//         }
//         
//         for(int i =0;i<4;i++){
//                 wheels[i].setPower(0);
//         }
//         motorA.setPower(aHoldPower);
//         sleep(1000);
//         
//         
//         //if(armUp)CheckSetArmHeight(armHeight,aLowPower,aHoldPower,aHighPower);
//     }
//     public void StrafeForSecondsBackwards(double p, double secs, boolean armUp){
//         runtime.reset();
//         while(runtime.seconds()<secs){
//             //fP balances backheavy weight distro
//             double fP = p;
//             double bP = p;//*.99;
//             for(int i =0;i<2;i++){
//                 wheels[i].setPower((i>0&&i<3)?fP-.1:-fP-.1);
//             }
//             for(int i =2;i<4;i++){
//                 wheels[i].setPower((i>0&&i<3)?bP-.1:-bP-.1);
//             }
//             if(armUp)CheckSetArmHeight(armHeight,aLowPower,aHoldPower,aHighPower);
//             telemetry.addLine(((Integer)motorA.getCurrentPosition()).toString());
//             telemetry.update();
//         }
//         
//         for(int i =0;i<4;i++){
//                 wheels[i].setPower(0);
//         }
//         motorA.setPower(aHoldPower);
//         sleep(1000);
//         
//         
//         //if(armUp)CheckSetArmHeight(armHeight,aLowPower,aHoldPower,aHighPower);
//     }
//     
//     public void DriveForSeconds(double p, double secs, boolean armUp){
//         runtime.reset();
//         while(runtime.seconds()<secs){
//             for(int i =0;i<4;i++){
//                     wheels[i].setPower(i<2?-p:p);
//             }
//             if(armUp)CheckSetArmHeight(armHeight,aLowPower,aHoldPower,aHighPower);
//         }
//         for(int i =0;i<4;i++){
//                 wheels[i].setPower(0);
//         }
//         motorA.setPower(aHoldPower);
//         sleep(1000);
//         
//         //if(armUp)CheckSetArmHeight(armHeight,aLowPower,aHoldPower,aHighPower);
//     }
//     
//     public void ClawSetClosed(boolean isClosed){
//         if (isClosed){
//             double closeGripLPosition = 0.35;
//             gripR.setPosition(closeGripLPosition);
//             gripL.setPosition(1-closeGripLPosition); //needs to turn the servos OFF after use
//                                                         //so they dont drain power
//         }
//         else{
//             gripL.setPosition(1);
//             gripR.setPosition(0);
//         }
//         sleep(1000);
//         gripR.close();
//         gripL.close();
//     }
//     
//     public void CheckSetArmHeight(int height, double lowP, double holdP, double highP){
//         int hDiff = motorA.getCurrentPosition()-height;
//         //if(java.lang.Math.abs(hDiff)>10){
//          //   motorA.setPower(hDiff>0?lowP:highP);
//         //}
//         //if(java.lang.Math.abs(hDiff)<1){
//             motorA.setPower(holdP);
//         //}
//     }
//     
//     public void checkPix(){
//         try
//         {
//             sleep(200);
//             CF = FQ.take();
//             telemetry.addData("AAAAAA", "AAAA");
//             //SSR = new CameraStreamSource();
//             //sleep(200);
//             CameraStreamServer.getInstance().setSource(vuforia);
//             sleep(200);
//             BM = vuforia.convertFrameToBitmap(CF);
//             int z = BM.getPixel(100,100);
//             //wait(2000);
//             int w = BM.getWidth();
//             int h = BM.getHeight();
//             int[] pixels = new int[w*h];
//             BM.getPixels(pixels,0,w,0,0,w,h);
//             
//             int wSection = w/3;
//             
//             int[] yellowCounts = new int[3];
//             for(int sec = 0; sec<3;sec++){
//                 for(int x =0;x<wSection;x++){
//                    for(int y =0;y<h;y++){
//                         
//                        
//                         
//                         int col = pixels[(y*w)+x+(wSection*sec)];
//                         int[] c = {Color.red(col),Color.green(col),Color.blue(col)};
//                         if(c[0]>140 && c[1]>140 && c[2]<90){
//                             yellowCounts[sec]++;
//                         }
//                     } 
//                 }
//             }
//             
//             int maxCount = 0;
//             int maxIndex=-1;
//             for(int i =0;i<3;i++){
//                 
//                 if(yellowCounts[i]>maxCount){
//                     maxCount =yellowCounts[i];
//                     maxIndex=i;
//                 }
//                 
//             }
//             duckSide=maxIndex;
//             
//             telemetry.addLine(
//                 ((Integer)yellowCounts[0]).toString()+", "
//                 +((Integer)yellowCounts[1]).toString()+", "
//                 +((Integer)yellowCounts[2]).toString()+", "
//                 +((Integer)duckSide).toString()
//                 );
//         }
//         catch(Exception e)
//         {
//             telemetry.addData("Nope", "BBBB");
//         }
//     }
// }
// 
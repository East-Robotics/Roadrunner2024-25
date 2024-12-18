package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class TeleOpMark extends LinearOpMode {

    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private DcMotor LSlide;
    private DcMotor RSlide;
    private DcMotor Arm;
    private Servo Wrist;
    private Servo Claw;


    boolean WristIsOpen = true;
    boolean ClawIsOpen = false;

    boolean lastYState = false;
    boolean currentYState = false;

    boolean lastXState = false;
    boolean currentXState = false;
    

    public void runOpMode() {
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LSlide = hardwareMap.get(DcMotor.class, "LSlide");
        RSlide = hardwareMap.get(DcMotor.class, "RSlide");
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");
        
        //Encoders
       /* LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        
     //   LArm.setDirection(DcMotor.Direction.REVERSE);        

        telemetry.addData("Status", "Running");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            currentYState = gamepad2.y;
            currentXState = gamepad2.x;

            double RFtgtPower = 0;
            double LFtgtPower = 0;
            double RBtgtPower = 0;
            double LBtgtPower = 0;
            double py = -gamepad1.left_stick_y;
            double px = -gamepad1.left_stick_x;
            double pa = gamepad1.right_stick_x;

            LFMotor.setPower(py - px + pa);
            LBMotor.setPower(-py - px - pa);
            RFMotor.setPower(py + px + pa);
            RBMotor.setPower(py + px - pa);

//Wrist toggle
            if (currentYState && !lastYState) {
                WristIsOpen = !WristIsOpen;
            }

            lastYState = currentYState;

            if (WristIsOpen) {
                Wrist.setPosition(0.4);
            }
             else {
                Wrist.setPosition(1);
            }



//Claw toggle
            if (currentXState && !lastXState) {
                ClawIsOpen = !ClawIsOpen;
            }

            lastXState = currentXState;

            if (ClawIsOpen) {
                Claw.setPosition(0.2);

            } else {
                Claw.setPosition(0.40);
            }

//Arm control
          if (gamepad2.right_bumper) {
                Arm.setPower(0.6);
            }
            else if(gamepad2.left_bumper){
                Arm.setPower(-0.6);
            }
            else{
                Arm.setPower(-0.07);
            }
//Slide control
            if (gamepad1.right_bumper){//down
                LSlide.setPower(0.7);
                RSlide.setPower(0.7);
            }
            else if (gamepad1.left_bumper){//up
                LSlide.setPower(-0.6);
                RSlide.setPower(-0.6);
            }
            else{
                LSlide.setPower(-0.07);
                RSlide.setPower(-0.07); 
            }
        }
    }
}

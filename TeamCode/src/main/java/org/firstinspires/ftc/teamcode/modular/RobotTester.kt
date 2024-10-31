package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

@Suppress("unused")
@TeleOp(name = "Robot Tester", group = "danger")
class RobotTester : OpMode() {
    private var robot = Robot(this.telemetry)

    // TODO: touch sensor for arm retracting so it can be controlled in auto / auto pose
    override fun init() {
        this.robot.initialize(this.hardwareMap)
        this.robot.registerButton(this.robot.BooleanButton(Gamepad::a, 0), Robot::switchDirection)
        this.robot.registerButton(
            this.robot.BooleanButton(Gamepad::b, 0),
            Robot::quarterSpeed
        )
        this.robot.registerDrivetrainButtons(
            this.robot.FloatButton(Gamepad::right_stick_y, 0),
            this.robot.FloatButton(Gamepad::right_stick_x, 0),
            this.robot.FloatButton(Gamepad::left_trigger, 0),
            this.robot.FloatButton(Gamepad::right_trigger, 0)
        )
        this.robot.registerArmButtons(
        )
    }

    override fun loop() {
        this.robot.tick(this.gamepad1, this.gamepad2)
        this.telemetry.update()
    }
}

package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.absoluteValue
import kotlin.reflect.KFunction0

class AutonomousRobot(
    private val telemetry: Telemetry,
    private val stopped: KFunction0<Boolean>,
    private val active: KFunction0<Boolean>
) {
    private lateinit var leftFront: DcMotor
    private lateinit var rightFront: DcMotor
    private lateinit var leftBack: DcMotor
    private lateinit var rightBack: DcMotor
    private lateinit var drivetrainMotors: Array<DcMotor>
    private lateinit var imu: IMU
    private var currentGamepad1 = Gamepad()
    private var pastGamepad1 = Gamepad()
    private var currentGamepad2 = Gamepad()
    private var pastGamepad2 = Gamepad()
    private val registeredBooleanInputs =
        HashMap<AutonomousRobot.BooleanButton, (Robot.BooleanState) -> Unit>()

    fun checkActive(): Boolean {
        return active() && !stopped()
    }

    fun initialize(hardwareMap: HardwareMap) {
        this.leftFront = hardwareMap.dcMotor["lfdrive"]
        this.rightFront = hardwareMap.dcMotor["rfdrive"]
        this.leftBack = hardwareMap.dcMotor["lbdrive"]
        this.rightBack = hardwareMap.dcMotor["rbdrive"]
        this.imu = hardwareMap["imu"] as IMU

        this.drivetrainMotors.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }

        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.FORWARD
        this.imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        )
        this.imu.resetYaw()
}


    private fun getHeading(): Float {
        return this.imu.getRobotOrientation(
            AxesReference.EXTRINSIC,
            AxesOrder.XZY,
            AngleUnit.DEGREES
        ).secondAngle
    }

    // headingOffset must be positive
    private fun turnUntilHeadingDelta(
        direction: TurnDirection,
        headingOffset: Double,
        speed: Double = 0.5
    ) {
        val firstHeading = this.getHeading()
        var lastHeading = firstHeading
        var change = 0.0
        while (change < headingOffset && this.checkActive()) {
            val currentHeading = getHeading()
            var delta = lastHeading - currentHeading
            if (currentHeading > 120 && lastHeading < -120) delta -= 360
            else if (currentHeading < -120 && lastHeading > 120) delta += 360
            change += delta.absoluteValue
            lastHeading = currentHeading
            this.turn(direction, speed)
        }
    }



    private fun turn(direction: TurnDirection, speed: Double = 0.2) {
        var mutableSpeed = speed
        if (direction == TurnDirection.LEFT) mutableSpeed *= -1
        this.leftFront.power = mutableSpeed
        this.rightFront.power = -mutableSpeed
        this.leftBack.power = mutableSpeed
        this.rightBack.power = -mutableSpeed
    }

    fun updateGamepads(gamepad1: Gamepad, gamepad2: Gamepad) {
        try {
            this.pastGamepad1.copy(this.currentGamepad1)
            this.pastGamepad2.copy(this.currentGamepad2)
            this.currentGamepad1.copy(gamepad1)
            this.currentGamepad2.copy(gamepad2)
        } catch (ignored: Exception) {
        }
    }

    private fun handleBooleanButtonTick(
        button: AutonomousRobot.BooleanButton,
        consumer: (Robot.BooleanState) -> Unit
    ) {
        val state = this.getState(button)
        consumer(state)
    }

    fun registerButton(button: AutonomousRobot.BooleanButton, consumer: () -> Unit) {
        val wrappingConsumer: (Robot.BooleanState) -> Unit = {
            if (it == Robot.BooleanState.RISING_EDGE) {
                consumer()
            }
        }

        this.registeredBooleanInputs[button] = wrappingConsumer
    }

    private fun getState(button: AutonomousRobot.BooleanButton): Robot.BooleanState {
        if (button.get() && !button.getPrev()) return Robot.BooleanState.RISING_EDGE
        else if (!button.get() && button.getPrev()) return Robot.BooleanState.FALLING_EDGE
        return Robot.BooleanState.INVARIANT
    }

    fun handleButtons() {
        this.registeredBooleanInputs.forEach { (button: AutonomousRobot.BooleanButton, consumer: (Robot.BooleanState) -> Unit) ->
            this.handleBooleanButtonTick(button, consumer)
        }
    }

    fun updateTelemetry() {
    }

    inner class BooleanButton(private val button: BooleanButtonType) {
        fun get(): Boolean {
            return this.button.get(this@AutonomousRobot.currentGamepad1)
        }

        fun getPrev(): Boolean {
            return this.button.get(this@AutonomousRobot.pastGamepad1)
        }
    }

 enum class Direction {
        FORWARD, BACKWARDS
    }

    enum class TurnDirection {
        LEFT, RIGHT
    }

    inner class ServoWrapper(
        private val servo: Servo,
        private val open: Double,
        private val closed: Double,
        private var state: Robot.ServoDualState = Robot.ServoDualState.CLOSED
    ) {
        fun set(state: Robot.ServoDualState) {
            this.state = state
            when (state) {
                Robot.ServoDualState.OPEN -> this.servo.position = this.open
                Robot.ServoDualState.CLOSED -> this.servo.position = this.closed
            }
        }

        fun set(position: Double) {
            this.servo.position = position
        }

        fun toggle() {
            this.set(this.state.opposite())
        }
    }

}

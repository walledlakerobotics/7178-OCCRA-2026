package frc.robot.subsystems.drivetrain

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import kotlinx.io.*
import net.jpountz.xxhash.XXHashFactory
import java.util.concurrent.atomic.AtomicBoolean

private const val kFrameSignature: UByte = 0xAAu
private const val kEscapeByte: UByte = 0xBBu

private const val kEscapeByteMask: UByte = 0x20u

private fun Sink.writeBoolean(value: Boolean) {
    writeUByte(if (value) 1u else 0u)
}

@Logged
class Pinpoint(val port: SerialPort.Port) : AutoCloseable {
    // port serial instance
    private var serialPort =
        SerialPort(115200, port)

    private var serialPortSim = if (RobotBase.isSimulation()) {
        com.fazecast.jSerialComm.SerialPort.getCommPorts().firstOrNull {
            it.vendorID == 0x2E8A && it.productID == 0xF00F
        }
    } else null

    @Volatile
    private var odometryData = OdometryData()

    private var repeatedZeroBytes = 0

    var connected = false
        private set

    private val isRunning = AtomicBoolean(false)

    private val buffer = Buffer()
    private val dataSequence = sequence {
        while (true) {
            // Find the start of the frame
            @Suppress("ControlFlowWithEmptyBody")
            while (buffer.readUByteYielding() != kFrameSignature);

            val lengthTag = buffer.unescape(4).readUIntLe()
            val payload = buffer.unescape(lengthTag.toLong())
            val checksumTag = buffer.unescape(4).readUIntLe()

            val factory = XXHashFactory.safeInstance()

            val checksum = factory.hash32()
                .hash(payload.peek().readByteArray(), 0, payload.size.toInt(), 0)
                .toUInt()

            if (checksum != checksumTag) {
                continue
            }

            odometryData = OdometryData(payload)
        }
    }.iterator()

    // updates in seprate thread in different intervals
    private val notifier = Notifier {
        if (!isRunning.compareAndSet(false, true)) {
            return@Notifier
        }

        if (RobotBase.isReal()) {
            if (serialPort.bytesReceived == 0) {
                connected = false

                repeatedZeroBytes++

                if (repeatedZeroBytes > 100) {
                    serialPort.close()

                    serialPort = SerialPort(115200, port)
                }

                isRunning.set(false)
                return@Notifier
            }

            connected = true
            repeatedZeroBytes = 0

            buffer.write(serialPort.read(serialPort.bytesReceived))
        } else {
            serialPortSim?.let { simPort ->
                try {
                    val bytes = ByteArray(simPort.bytesAvailable())

                    simPort.readBytes(bytes, bytes.size)

                    buffer.write(bytes)
                    
                    connected = true
                } catch (_: Exception) {
                    connected = false
                    
                    simPort.closePort()

                    serialPortSim = com.fazecast.jSerialComm.SerialPort.getCommPorts().firstOrNull {
                        it.vendorID == 0x2E8A && it.productID == 0xF00F
                    }

                    serialPortSim?.openPort()

                    isRunning.set(false)
                    return@Notifier
                }
            }
        }

        try {
            dataSequence.next()
        } finally {
            isRunning.set(false)
        }
    }

    var position
        get() = odometryData.position
        set(value) = resetPosition(value)

    var heading
        get() = position.rotation!!
        set(value) = resetPosition(Pose2d(position.translation, value))

    val rotation
        get() = odometryData.rotation

    val velocity: ChassisSpeeds
        get() = odometryData.velocity

    init {
        serialPortSim?.apply {
            baudRate = 115200
            numDataBits = 8
            parity = com.fazecast.jSerialComm.SerialPort.NO_PARITY
            numStopBits = com.fazecast.jSerialComm.SerialPort.ONE_STOP_BIT

            openPort()
        }

        notifier.startPeriodic(0.01)

        SmartDashboard.putData("Odometry/ResetOdometry", Commands.runOnce({
            resetPosition(Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))
        }).ignoringDisable(true))
    }

    context(scope: SequenceScope<Unit>)
    private suspend fun Source.unescape(count: Long): Buffer {
        val escapedBuffer = Buffer()

        while (escapedBuffer.size < count) {
            var byte = readUByteYielding()

            if (byte == kEscapeByte) {
                val nextByte = readUByteYielding()

                byte = nextByte xor kEscapeByteMask
            }

            escapedBuffer.writeUByte(byte)
        }

        return escapedBuffer
    }

    context(scope: SequenceScope<Unit>)
    private suspend fun Source.readUByteYielding(): UByte {
        while (exhausted()) scope.yield(Unit)
        return readUByte()
    }

    private fun writeFrame(payload: Buffer) {
        val buffer = Buffer()

        val escapedPayload = payload.peek().escape()

        val lengthTag = Buffer().apply { writeUIntLe(payload.size.toUInt()) }.escape()

        val factory = XXHashFactory.safeInstance()
        val checksum = factory.hash32().hash(payload.peek().readByteArray(), 0, payload.size.toInt(), 0).toUInt()

        val checksumTag = Buffer().apply { writeUIntLe(checksum) }.escape()

        buffer.writeUByte(kFrameSignature)
        buffer.write(lengthTag, lengthTag.size)
        buffer.write(escapedPayload, escapedPayload.size)
        buffer.write(checksumTag, checksumTag.size)

        val bytes = buffer.readByteArray()

        if (RobotBase.isReal()) {
            serialPort.write(bytes, bytes.size)
        } else {
            serialPortSim?.writeBytes(bytes, bytes.size)
        }
    }

    private fun Source.escape(): Buffer {
        val escapedBuffer = Buffer()

        while (!exhausted()) {
            val byte = readUByte()

            if (byte == kEscapeByte || byte == kFrameSignature) {
                escapedBuffer.writeUByte(kEscapeByte)
                escapedBuffer.writeUByte(byte xor kEscapeByteMask)

                continue
            }

            escapedBuffer.writeUByte(byte)
        }

        return escapedBuffer
    }

    fun recalibrateIMU() {
        val data = ResetData.recalibrateIMU()
        writeFrame(data.toBuffer())
    }

    fun resetPosition(position: Pose2d) {
        val data = ResetData.resetPosition(position)
        writeFrame(data.toBuffer())
    }

    fun setEncoderOffsets(offsetX: Distance, offsetY: Distance) {
        val data = ResetData.setEncoderOffsets(offsetX, offsetY)
        writeFrame(data.toBuffer())
    }

    fun setEncoderOffsets(offsetX: Double, offsetY: Double) =
        setEncoderOffsets(Meters.of(offsetX), Meters.of(offsetY))

    fun setEncoderDirections(xDirection: EncoderDirection, yDirection: EncoderDirection) {
        val data = ResetData.setEncoderDirections(xDirection, yDirection)
        writeFrame(data.toBuffer())
    }
    
    fun setEncoderResolution(ticksPerMM: Double) {
        val data = ResetData.setEncoderResolution(ticksPerMM.toFloat())
        writeFrame(data.toBuffer())
    }

    override fun close() {
        serialPort.close()
        serialPortSim?.closePort()

        notifier.close()
    }
}

@Logged
private data class OdometryData(
    val encoderX: UInt = 0u,
    val encoderY: UInt = 0u,

    val positionX: Double = 0.0,
    val positionY: Double = 0.0,
    val positionHeading: Double = 0.0,

    val quaternionW: Double = 0.0,
    val quaternionX: Double = 0.0,
    val quaternionY: Double = 0.0,
    val quaternionZ: Double = 0.0,

    val velocityX: Double = 0.0,
    val velocityY: Double = 0.0,
    val velocityHeading: Double = 0.0
) {
    val position: Pose2d
        get() = Pose2d(
            Translation2d(Millimeters.of(positionX), Millimeters.of(positionY)),
            heading
        )

    val heading: Rotation2d
        get() = Rotation2d.fromRadians(positionHeading)

    val rotation: Rotation3d
        get() = Rotation3d(quaternion)

    val quaternion: Quaternion
        get() = Quaternion(quaternionW, quaternionX, quaternionY, quaternionZ)

    val velocity: ChassisSpeeds
        get() = ChassisSpeeds(
            Millimeters.per(Second).of(velocityX),
            Millimeters.per(Second).of(velocityY),
            DegreesPerSecond.of(velocityHeading)
        )

    val velocityFieldRelative: ChassisSpeeds
        get() = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, position.rotation)

    constructor(
        encoderX: UInt,
        encoderY: UInt,
        positionX: Float,
        positionY: Float,
        positionHeading: Float,
        quaternionW: Float,
        quaternionX: Float,
        quaternionY: Float,
        quaternionZ: Float,
        velocityX: Float,
        velocityY: Float,
        velocityHeading: Float
    ) : this(
        encoderX,
        encoderY,
        positionX.toDouble(),
        positionY.toDouble(),
        positionHeading.toDouble(),
        quaternionW.toDouble(),
        quaternionX.toDouble(),
        quaternionY.toDouble(),
        quaternionZ.toDouble(),
        velocityX.toDouble(),
        velocityY.toDouble(),
        velocityHeading.toDouble()
    )

    constructor(buffer: Buffer) : this(
        buffer.readUIntLe(),
        buffer.readUIntLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe()
    )
}

@ConsistentCopyVisibility
private data class ResetData private constructor(
    val calibrateIMU: Boolean = false,

    val setPosition: Boolean = false,
    val positionX: Float = 0.0f,
    val positionY: Float = 0.0f,
    val positionHeading: Float = 0.0f,

    val setOffsets: Boolean = false,
    val offsetX: Float = 0.0f,
    val offsetY: Float = 0.0f,

    val setDirections: Boolean = false,
    val xInverted: Boolean = false,
    val yInverted: Boolean = false,
                              
    val setResolution: Boolean = false,
    val resolution: Float = 1.0f
) {
    fun toBuffer(): Buffer {
        val buffer = Buffer()

        buffer.writeBoolean(calibrateIMU)

        buffer.writeBoolean(setPosition)
        buffer.writeFloatLe(positionX)
        buffer.writeFloatLe(positionY)
        buffer.writeFloatLe(positionHeading)

        buffer.writeBoolean(setOffsets)
        buffer.writeFloatLe(offsetX)
        buffer.writeFloatLe(offsetY)

        buffer.writeBoolean(setDirections)
        buffer.writeBoolean(xInverted)
        buffer.writeBoolean(yInverted)
        
        buffer.writeBoolean(setResolution)
        buffer.writeFloatLe(resolution)

        return buffer
    }

    companion object {
        fun recalibrateIMU() = ResetData(
            calibrateIMU = true,
            setPosition = false,
            positionX = 0.0f,
            positionY = 0.0f,
            positionHeading = 0.0f,
            setOffsets = false,
            offsetX = 0.0f,
            offsetY = 0.0f,
            setDirections = false,
            xInverted = false,
            yInverted = false
        )

        fun resetPosition(position: Pose2d) = ResetData(
            calibrateIMU = false,
            setPosition = true,
            positionX = position.measureX.`in`(Millimeters).toFloat(),
            positionY = position.measureY.`in`(Millimeters).toFloat(),
            positionHeading = position.rotation.radians.toFloat(),
            setOffsets = false,
            offsetX = 0.0f,
            offsetY = 0.0f,
            setDirections = false,
            xInverted = false,
            yInverted = false
        )

        fun setEncoderOffsets(offsetX: Distance, offsetY: Distance) = ResetData(
            calibrateIMU = false,
            setPosition = false,
            positionX = 0.0f,
            positionY = 0.0f,
            positionHeading = 0.0f,
            setOffsets = true,
            offsetX = offsetX.`in`(Millimeters).toFloat(),
            offsetY = offsetY.`in`(Millimeters).toFloat(),
            setDirections = false,
            xInverted = false,
            yInverted = false
        )

        fun setEncoderDirections(xDirection: EncoderDirection, yDirection: EncoderDirection) = ResetData(
            calibrateIMU = false,
            setPosition = false,
            positionX = 0.0f,
            positionY = 0.0f,
            positionHeading = 0.0f,
            setOffsets = false,
            offsetX = 0.0f,
            offsetY = 0.0f,
            setDirections = true,
            xInverted = xDirection.inverted,
            yInverted = yDirection.inverted
        )
        
        fun setEncoderResolution(ticksPerMM: Float) = ResetData(
            calibrateIMU = false,
            setPosition = false,
            positionX = 0.0f,
            positionY = 0.0f,
            positionHeading = 0.0f,
            setOffsets = false,
            offsetX = 0.0f,
            offsetY = 0.0f,
            setDirections = false,
            xInverted = false,
            yInverted = false,
            setResolution = true,
            resolution = ticksPerMM
        )
    }
}

enum class EncoderDirection(val inverted: Boolean) {
    FORWARD(false),
    BACKWARD(true)
}
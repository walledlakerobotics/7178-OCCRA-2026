package org.walledlakerobotics.pinpointrio

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.SerialPort
import kotlinx.io.*
import net.jpountz.xxhash.XXHashFactory
import java.util.concurrent.atomic.AtomicBoolean

private typealias JSerialPort = com.fazecast.jSerialComm.SerialPort

private const val kFrameSignature: UByte = 0xAAu
private const val kEscapeByte: UByte = 0xBBu

private const val kEscapeByteMask: UByte = 0x20u

private fun Sink.writeBoolean(value: Boolean) {
    writeUByte(if (value) 1u else 0u)
}

class Pinpoint(val port: SerialPort.Port) : AutoCloseable {
    // port serial instance
    private var serialPort: SerialPort? = null

    private var serialPortSim: JSerialPort? = null

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

    // updates in separate thread in different intervals
    private val notifier = Notifier {
        if (!isRunning.compareAndSet(false, true)) {
            return@Notifier
        }

        if (RobotBase.isReal()) {
            if (serialPort == null || serialPort?.bytesReceived == 0) {
                repeatedZeroBytes++

                if (repeatedZeroBytes > 50) {
                    repeatedZeroBytes = 0

                    resetSerial()
                }

                isRunning.set(false)
                return@Notifier
            }

            connected = true
            repeatedZeroBytes = 0
            
            val bytes = serialPort?.read(serialPort!!.bytesReceived) ?: byteArrayOf()

            buffer.write(bytes)
        } else {
            try {
                val bytes = ByteArray(serialPortSim!!.bytesAvailable())

                serialPortSim!!.readBytes(bytes, bytes.size)

                connected = true
                buffer.write(bytes)
            } catch (_: Exception) {
                resetSerial()

                isRunning.set(false)
                return@Notifier
            }
        }

        try {
            dataSequence.next()
        } finally {
            isRunning.set(false)
        }
    }

    init {
        resetSerial()

        notifier.startPeriodic(0.01)
    }

    private fun resetSerial() {
        DriverStation.reportError("Pinpoint not connected", false)
        
        connected = false

        if (RobotBase.isReal()) {
            serialPort = try {
                serialPort?.close()

                SerialPort(115200, port)
            } catch(_: Exception) { null }
        } else {
            serialPortSim?.closePort()

            serialPortSim = JSerialPort.getCommPorts().firstOrNull {
                it.vendorID == 0x2E8A && it.productID == 0xF00F
            }

            serialPortSim?.apply {
                baudRate = 115200
                numDataBits = 8
                parity = JSerialPort.NO_PARITY
                numStopBits = JSerialPort.ONE_STOP_BIT

                openPort()
            }
        }
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
            serialPort?.write(bytes, bytes.size)
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

    val encoderX
        get() = odometryData.encoderX

    val encoderY
        get() = odometryData.encoderY

    var position
        get() = odometryData.position
        set(value) = resetPosition(value)

    var rotation2d
        get() = position.rotation!!
        set(value) = resetPosition(Pose2d(position.translation, value))

    val rotation3d
        get() = odometryData.rotation3d

    val velocity: ChassisSpeeds
        get() = odometryData.velocity

    val velocityFieldRelative = odometryData.velocityFieldRelative

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
        serialPort?.close()
        serialPortSim?.closePort()

        notifier.close()
    }
}

private data class OdometryData(
    val encoderX: Int = 0,
    val encoderY: Int = 0,

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
            rotation2d
        )

    val rotation2d: Rotation2d
        get() = Rotation2d.fromRadians(positionHeading)

    val rotation3d: Rotation3d
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
        encoderX: Int,
        encoderY: Int,
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
        buffer.readIntLe(),
        buffer.readIntLe(),
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
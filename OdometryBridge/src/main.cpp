#include <Arduino.h>
#include <goBILDA_Pinpoint.h>
#include <writing.hpp>
#include <reading.hpp>
#include <numbers>

goBILDA::Pinpoint pinpoint = goBILDA::Pinpoint();

void setup()
{
  // port
  Serial.begin(115200);

  Wire.setClock(400000); // 400kHz I2C
  Wire.setTimeout(10); // 10ms timeout for I2C operations

  // configurating pinpoint
  pinpoint.begin();

  delay(2000);
}

void loop()
{
  while (rp2040.fifo.available()) {
    reset_data* data_ptr = reinterpret_cast<reset_data*>(rp2040.fifo.pop());

    if (data_ptr->calibrate_imu) {
      pinpoint.recalibrateIMU();
    }

    if (data_ptr->set_position) {
      pinpoint.setPosition(goBILDA::Pose2D(data_ptr->position_x, data_ptr->position_y, data_ptr->position_heading));
    }

    if (data_ptr->set_offsets) {
      pinpoint.setOffsets(data_ptr->offset_x, data_ptr->offset_y);
    }

    if (data_ptr->set_directions) {
      pinpoint.setEncoderDirections(
        data_ptr->x_inverted ? goBILDA::EncoderDirection::Backward : goBILDA::EncoderDirection::Forward,
        data_ptr->y_inverted ? goBILDA::EncoderDirection::Backward : goBILDA::EncoderDirection::Forward
      );
    }

    if (data_ptr->set_resolution) {
      pinpoint.setEncoderResolution(data_ptr->resolution);
    }

    delete data_ptr;
  }

  goBILDA::BulkReadData bulk_data = pinpoint.bulkRead();

  p_data position_data = p_data();
  position_data.encoder_x = bulk_data.EncoderX;
  position_data.encoder_y = bulk_data.EncoderY;
  position_data.position_x = bulk_data.Position.x;
  position_data.position_y = bulk_data.Position.y;
  position_data.position_heading = bulk_data.Position.heading;
  position_data.quaternion_w = pinpoint.getQuaternion().w;
  position_data.quaternion_x = pinpoint.getQuaternion().x;
  position_data.quaternion_y = pinpoint.getQuaternion().y;
  position_data.quaternion_z = pinpoint.getQuaternion().z;
  position_data.velocity_x = bulk_data.VelocityX;
  position_data.velocity_y = bulk_data.VelocityY;
  position_data.velocity_heading = bulk_data.VelocityH;

  // converts data into a buffer of bytes.

  Buffer payload = position_data.to_bytes();

  Buffer buffer = create_frame(payload);
  size_t buffer_size = buffer.size();

  // sends data and waits for it to complete.
  Serial.write(buffer.data(), buffer_size);

  delay(5);
}

void setup1()
{
  while (!Serial) {
    delay(50);
  }

  SerialBuffer buffer = SerialBuffer(Serial);

  while (true) {
    Buffer payload;

    if (read_frame(buffer, payload)) {
      reset_data* data_ptr = new reset_data(reset_data::from_bytes(payload));

      rp2040.fifo.push(reinterpret_cast<uint32_t>(data_ptr));
    }

    yield();
    delay(10);
  }
}

void loop1() {}
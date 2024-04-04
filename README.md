# whi_rc_bridge
Bridging Remote Controller and velocity command(geometry_msgs/Twist). Currently, both IIC and SBUS transportation are supported.

Although this bridge is designed to support most Remote Controllers, like RadioLink, further revision might be needed to adapt to other specific RC receivers.

## Dependencies
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Params
```
whi_rc_bridge:
  frequency: 20.0
  twist_topic: cmd_vel
  max_linear: 1.0 #m/s
  max_angular: 1.57 #rad/s
  damp_angular: true
  rc_state_topic: rc_state
  cancel_goal_topic: move_base/cancel
  # for T8S(BT)
  # channels_name: ["left_right", "forth_back", "throttle", "active"]
  # channels_offset: [-1, -10, 2, -40]
  # for T8FB(BT)
  channels_name: ["left_right", "forth_back", "throttle", "dummy", "dummy", "clear_error", "active"]
  channels_offset: [-1, -10, 2, 0, 0, 0, -40]
  print_raw: false
  hardware: serial
  i2c:
    bus_addr: 1
    device_addr: 8
  serial:
    device: /dev/ttyUSB0
```

## Related to SBUS
SBUS is a bus protocol for receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single serial line can be connected with up to 16 servos with each receiving a unique command.

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:

Byte[0]: SBUS header, 0x0F

Byte[1 -22]: 16 servo channels, 11 bits each

Byte[23]

Bit 0: channel 17 (0x01)

Bit 1: channel 18 (0x02)

Bit 2: frame lost (0x04)

Bit 3: failsafe activated (0x08)

Byte[24]: SBUS footer

Note that a lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that many frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms or 20 ms, depending on the system configuration.

A variation on SBUS called "Fast SBUS" has started to be used. This uses a baudrate of 200000 and a quicker update rate.

Note on CH17 and CH18: channel 17 and channel 18 are digital on/off channels. These are not universally available on all SBUS receivers and servos.

### Invert circuit
SBUS signal should be connected to the RX pin of the UART, and there should be an inverter in place for the inverted SBUS signal. Like the following figure:

![sbus_inverter](https://github.com/xinjuezou-whi/whi_rc_bridge/assets/72239958/752f75bc-8607-487a-a5cd-df261deace31)

or you can make one with a transistor and two resistors.

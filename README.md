# whi_rc_bridge
Bridging Remote Controller and velocity command(geometry_msgs/Twist). Currently, only IIC transportation is supported. Further S.BUS based on serial will be introduced.

Although this bridge is designed to support most Remote Controllers, like RadioLink, further revision might be needed to adapt to other specific RC receivers.

## Params
```
whi_rc_bridge:
  frequency: 20.0
  twist_topic: /cmd_vel
  max_linear: 1.0 #m/s
  max_angular: 1.57 #rad/s
  damp_angular: true
  channels_name: ["left_right", "forth_back", "throttle", "active"]
  channels_offset: [-1, -10, 2, 0]
  hardware: i2c
  i2c:
    bus_addr: 1
    device_addr: 8
```

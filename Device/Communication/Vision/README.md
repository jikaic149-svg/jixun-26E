# Device/Communication/Vision

This hardware-independent Device binds the packet and reliable-link
algorithms into a vision communication state machine. UART ownership and the
meaning of received data remain in the caller.

Wire format:

```text
@1|TYPE|SEQ|LEN|PAYLOAD|CRC16\n
```

Supported reliable commands are `START`, `END`, `REQUEST`, and `CONTEXT`.
`CONTEXT` carries an application-defined unsigned 16-bit value. The Device
layer does not interpret it.

Two telemetry formats are accepted:

- Legacy `RESULT`: color and shape.
- `DETECTION`: sender session, context, class ID, shape, confidence in the
  range 0..1000, and image center coordinates.

Critical commands use ACK, retry, reboot-session duplicate suppression, and a
dedicated eight-entry event queue. Detection telemetry is latest-wins and does
not consume that queue. Motion decisions never originate in this Device; Task
code remains the final arbiter.

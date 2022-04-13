---
sort: 1
---

# Create® 3

On the TurtleBot 4, the connection of the User Interface board with the Create® 3 robot is only through the VBAT connector J7, which uses <span style="color:blue">JST XH-style</span> connector. The connector has been marked with Positive and Negative signs on the board (Positive being pin 1). The VBAT line is fused with a PTC fuse rated at 2A. 

<figure>
    <img src="media/create_adapter.png" alt="Create® 3 Power Adapter" style="width: 58%; margin-right: 5%; margin-left: 5%;"/>
    <img src="media/UI_PWR.png" alt="TurtleBot 4 UI Power" style="width: 20%"/>
    <figcaption>Create® 3 Power Adapter (left) and J7 connector (right)</figcaption>
</figure>

The Create® 3 power adapter also supplies the Raspberry Pi 4 with power and communication through a <span style="color:red">USB 2.0 (Type C)</span> on both the TurtleBot 4 and TurtleBot 4 Lite. The USB interface can supply up to 3A at 5V.

For more details, visit the [Create® 3 Documentation](https://iroboteducation.github.io/create3_docs/hw/electrical/).

```warning
It is recommended to not drain the robot below 20% as VBAT voltage begin to decline sharply at this level.
```
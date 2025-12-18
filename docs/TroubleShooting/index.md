### Troubleshooting

#### Engine Crashes Repeatedly on Startup

If the V2X simulator crashes repeatedly during launch, try clearing temporary and cached data:

1. **Fully close** the application (and Unity Editor, if applicable).
2. Navigate to your simulator’s root directory (or Unity project folder if running from source).
3. **Delete the following folders** (if they exist):
   - `Logs/` *(in your first try only remove the logs)*
   - `Library/`
   - `Temp/`
4. Relaunch the simulator.

> **Note:**  
> Unity regenerates these folders automatically on startup. Removing them forces a clean reimport of assets and clears corrupted cache files, which often resolves persistent crashes or startup failures.


#### Stop Lines Don't Work with Traffic Lines
- **StopLine Reference Not Assigned**  
  The TrafficLane component requires a StopLine reference, and the referenced StopLine GameObject must have a TrafficLight component. If either condition is not met, the code will skip processing the lane entirely.

- **Waypoint Positioning**  
  The first waypoint of your approach lane must be positioned **at or before** the physical stop line. If the waypoint is placed after the stop line, vehicles will cross the stop line before stopping.

- **Lane Connectivity (most important reason)**  
  The approach lane must include the intersection lane in its "Next Lanes" array. Without this connection, vehicles cannot "look ahead" to detect the traffic light state.

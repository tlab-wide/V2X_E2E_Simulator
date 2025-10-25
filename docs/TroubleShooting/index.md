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


# Camera View Tab

The **View** panel lets you switch the main scene camera between several viewpoints at run time, directly from the on‑screen UI (inside the right‑side settings canvas).

There are three built‑in views:

| # | View | Description |
|---|------|-------------|
| 1 | **Follow** | The default chase camera that follows the ego vehicle (the existing `FollowCamera`). |
| 2 | **Top View** | A fixed bird's‑eye camera looking down on the intersection (`Camera Top View Int#1`). |
| 3 | **Free Camera** | A movable fly‑camera you can drive anywhere in the scene. |

Only one view is active at a time. When a view is selected, its `Camera`, `AudioListener` and optional control script are enabled and every other view's are disabled, so there is always exactly one rendering camera and one audio listener.

## Using the panel

- Click a button in the **View** panel, or press the number keys **1 / 2 / 3** to switch views.
- Press **V** to show/hide the panel.

## Free Camera controls

When the **Free Camera** view is active you can move it freely:

| Input | Action |
|-------|--------|
| Right‑mouse drag | Look around (pitch / yaw) |
| W / A / S / D | Move forward / left / back / right |
| Q / E | Move down / up |
| Left Shift (hold) | Move faster (sprint) |
| Scroll wheel | Dolly forward / back |

The Free Camera spawns at the Follow camera's current position and uses unscaled time, so it keeps responding even when the simulation is paused or slowed.

## Components

The feature is driven by two scripts under `Assets/V2X/Script/UI/`:

- **`CameraViewManager`** — holds the list of views, switches the active camera, highlights the active button and toggles the panel. `SelectView(int)` is the method wired to each button's `OnClick`. If `Auto Create Movable Camera` is enabled, the Free Camera is created automatically at play time (copying the Follow camera's render settings).
- **`FreeFlyCamera`** — the fly‑camera controller used by the Free Camera view. Movement/look speeds are exposed as `MoveSpeed`, `SprintMultiplier`, `ScrollSpeed` and `LookSensitivity`.

### Adding another view

1. Add the camera to the `Views` list on the `CameraViewManager` component (set its `Label`, `Camera`, and optionally a `ControlBehaviour` to enable only while active).
2. Add a button to the **View** panel and wire its `OnClick` to `CameraViewManager.SelectView` with the new view's index.
3. Add the button to the manager's `UI Buttons` list (same order as `Views`) so it highlights when active.

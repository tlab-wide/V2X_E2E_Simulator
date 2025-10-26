# Teleporter Component

## Overview

The Teleporter component creates a portal system in Unity that instantly transports ego vehicles to a designated location. 


> **Note:** This component only affects ego vehicles with the V2X Vehicle component. Normal traffic cars are not affected.

---

## How It Works

The teleporter operates through a simple but effective mechanism:

1. **Detection** - Uses Unity's trigger collision system to detect vehicle entry
2. **Validation** - Searches for the V2X Vehicle component in the colliding object's hierarchy
3. **Teleportation** - Instantly relocates the vehicle to the destination pivot's position and rotation
4. **Reset** - Vehicle transform matches the destination without physics artifacts

---

## Setup Guide

### Step 1: Create the Entrance Trigger

- Create an empty GameObject
- Add a Collider component (Box, Sphere, or Capsule)
- Enable the **"Is Trigger"** checkbox
- Scale the collider to your desired entrance area
- Attach the **Teleporter script** to this GameObject

### Step 2: Create the Exit Point

- Create another empty GameObject (this will be your exit pivot)
- Position it where vehicles should appear after teleportation
- Orient this pivot carefully - vehicles will face its forward direction

### Step 3: Connect the Components

- Select the entrance trigger GameObject
- In the Inspector, assign the exit GameObject to the **"Teleporter Pivot"** field

---

<div align="center"> 
    <img src="image.png" alt="Teleporter Component Setup" style="max-width: 100%; height: auto;" /> <br /> 
    <em>Figure 1: The final position after teleport</em> 
</div>
<div align="center"> 
    <img src="image-1.png" alt="Scene Setup" style="max-width: 100%; height: auto;" /> <br /> 
    <em>Figure 2: Teleport gate and the box collider</em> 
</div>

---

## Usage Notes

### Requirements
- Only affects GameObjects with the **V2X Vehicle component**
- Trigger collider must be appropriately sized
- Position trigger to activate at the right moment

### Best Practices
- **Exit Pivot Rotation** - Determines vehicle facing direction after teleportation
- **Ground Level Placement** - Place exit pivot at or slightly above ground level
- **Testing** - Test thoroughly to ensure smooth transitions

### Important Reminders
- Does not affect normal traffic or AI vehicles
- Teleportation is instant with no transition delay
- Adjust trigger size for optimal activation timing

---

## Use Cases

- **Loop Tracks** - Create endless racing circuits
- **Testing Scenarios** - Quickly reset vehicle positions
- **Level Transitions** - Seamless world transitions
- **Checkpoint Systems** - Create advanced gameplay mechanics

---

*Documentation for Unity Teleporter Component*
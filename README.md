# Mini-Fork (BluePad Edition) — by Jakezilla

This is a fork of the awesome [Mini-Fork by Professor Boots](https://github.com/ProfessorBoots/Mini-Fork). I can never leave anything alone, so I set out to refine the control feel and add a few features that suit my setup better.

Currently, this fork focuses on improvements to the **BluePad** version.

## 🔧 Key Changes

- **Expo curves & deadband tuning**  
  Improved control feel, especially for low-cost PS4 controllers. This helps smooth out twitchy inputs and gives better low-speed precision.

- **Half-speed mode toggle**  
  Designed to tame 200RPM (or faster) motors. I found 100RPM too slow, and 200RPM lacked fine control — this toggle gives the best of both worlds.  
  Toggle is activated by the **A button** (mapped to **X on DualShock**), with rumble and LED feedback if supported.

- **Servo control improvements**  
  Servo updates are now handled via a timed loop rather than reacting to every incoming controller packet. This improves smoothness and longevity, especially with non-polling controllers like the Xbox One.  
  Additionally, the **mast servo** now uses **microsecond-level control** (`uS`) for finer resolution and more precise positioning — ideal for scale realism or nuanced articulation.

- **LED feedback for PS4 controllers**  
  Added support for LED control to enhance visual feedback and controller integration.

- **Other small tweaks and refinements**  
  Various minor edits to improve usability, maintainability, and responsiveness.

---

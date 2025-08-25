# Mini-Fork (BluePad Edition) — by Jakezilla

This is a fork of the awesome [Mini-Fork by Professor Boots](https://github.com/ProfessorBoots/Mini-Fork). I can never leave anything alone, so I set out to refine the control feel and add a few features that suit my setup better.

Currently, this fork focuses on improvements to the **BluePad** version and **3D File Mods**.

## 🔧 Key Changes to BluePad version

- **Expo curves & deadband tuning**  
  Improved control feel, especially for low-cost PS4 controllers. This helps smooth out twitchy inputs and gives better low-speed precision.

- **Half-speed mode toggle**  
  Designed to tame 200RPM (or faster) motors. I found 100RPM too slow, and 200RPM lacked fine control — this toggle gives the best of both worlds.  
  Toggle is activated by the **A button** (mapped to **X on DualShock**), with rumble and LED feedback if supported.

- **Servo control improvements**  
  Servo updates are now handled via a timed loop rather than reacting to every incoming controller packet. This improves smoothness and longevity, especially with non-polling controllers like the Xbox One.  
  Additionally, the **mast servo** now uses **microsecond control** for finer resolution and more precise positioning — ideal for scale realism or nuanced articulation.

- **LED feedback for PS4 controllers**  
  Added support for LED control to enhance visual feedback and controller integration.

- **Other small tweaks and refinements**  
  Various minor edits to improve usability, maintainability, and responsiveness.

---

## 🧱 3D Printed File Modifications

I’ve also made and collected several modifications to the 3D printed parts to enhance durability, ease of use, and visual refinement:

- **Orca Slicer Project**  
  <img width="1609" height="869" alt="Orca Slicer Project" src="https://github.com/user-attachments/assets/a55f7b48-adf0-474b-aaba-c6f17cb6786d" />  
  Orca Slicer project with all the modded files, support blockers, dual-color options, and reminder notes for support settings. I deviated slightly from the original primary/secondary/highlight color scheme to better suit my build.

- **Lower Interior with ON Switch Direction**  
  <img width="725" height="473" alt="Lower Interior with ON Switch Direction" src="https://github.com/user-attachments/assets/0fa17433-307a-4e87-80d7-bf924604c69b" />  
  This interior part includes directional labeling for the ON switch. Available in dual-color or single-color indented versions.

- **Mast Motor Cover**  
  <img width="510" height="466" alt="Mast Motor Cover" src="https://github.com/user-attachments/assets/c8b1c7cf-f07b-498d-9048-2fde0ac4d35f" />  
  A redesigned cover that shields the mast motor’s exposed gears. Helps prevent debris intrusion and adds a cleaner, more finished look.

- **Main Body Rear Pivot Hole Pre-Drill**  
  <img width="485" height="357" alt="Main Body Rear Pivot Hole Pre-Drill" src="https://github.com/user-attachments/assets/4f26ac2f-7dbf-4aa4-9f0c-2ba247cf6858" />  
  Modified the main body STL so the **WDR** rear pivot hole is pre-modeled — no need to drill after printing. This improves alignment and saves post-processing time.

- **Rear Steering Pivot Hole Mod (WDR)**  
  Includes the **WDR** rear steering pivot files and drill guide. These allow for better articulation and durability in the rear axle setup.

- **Fender Mods (WDR)**  
  Includes **WDR’s** fender modifications for improved rear tire clearance and a more aggressive stance.

---

  ## 🚧 Possible Future Ideas


###  Low Voltage Monitoring
- Integrate battery voltage sensing with ADC averaging and debounce logic.
  
###  FlySky iBUS Support
- Add support for FlySky transmitters via iBUS interface.

###  PCB Improvements
- **RX2/TX2 Header Access**  
  Add labeled header pins for RX2 and TX2 on the ESP32 to simplify serial debugging or telemetry modules.

- **Motor Connection Pads**  
  Include large solder pads with anchor vias as an alternative to screw terminals — ideal for high-current motor leads or compact builds.

- **Header Pin Spacing**  
  Move header pins closer together to make hand-soldering easier and reduce board footprint.

---

If you’ve got ideas, mods, or feedback — feel free to reach out or fork and experiment.


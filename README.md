# Find a Feasible Mobile Manipulation

Find a feasible pose of robot base for good manipulability

1. Offline
   1. Configuration
      1. `M`: X-Y-Theta manipulability map
      2. `Rsize`: the size of the robot-base
   1. Preparation
      1. Convert the `M` to a feasibility map `F` (helical shape).
      2. ~~[SKIP] Convert a cartesian grid `F` to a cylindrical grid `F` by linear interpolation.~~
2. Online
   1. Input
      1. `Pt`: Position of the target object (relative to the robot’s current pose)
      2. `Obs`: Area list of ground obstacles
      3. `Cr`: Constraints on the approach angle (relative to the robot heading)
      4. `Ct`: Constraints on the approach angle (relative to the target heading)
   2. Process
      1. Cut the range of `Cr` from `F` and set it to `Fcut`.
      2. ~~[SKIP] Scan the maximum points for the radius and angle by each circle in `Fcut`.~~
      3. Wipe the `Fcut` in the range of `Ct`.
      4. And extract only the maximum as a `Fmax`.
      5. Remove all obstacle areas from `Fmax` with the offset of `Rsize`.
   3. Output
      1. Candidate poses (sorted in descending order of manipulability)

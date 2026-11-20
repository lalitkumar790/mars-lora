# Crazyflie 2 model attribution

Source: Google DeepMind MuJoCo Menagerie / bitcraze_crazyflie_2/cf2.xml
https://github.com/google-deepmind/mujoco_menagerie/tree/main/bitcraze_crazyflie_2

License: MIT (upstream Crazyflie 2 Menagerie model; verify and retain the repository's license terms before redistribution of any additional meshes).
The full upstream XML is preserved for study in cf2_upstream.xml. It depends on mesh assets not packaged in this prototype.

The browser's `public/crazyflie.xml` is a **simplified derived dynamics-compatible model** using the upstream 27-gram mass and diagonal moments of inertia but using primitives and four thrust sites instead of the upstream integrated thrust/moment actuators. It is not a bit-for-bit rendering or independently calibrated copy of the reference model. Both variants use real MuJoCo rigid-body physics.

Source upstream MIT license: https://github.com/google-deepmind/mujoco_menagerie/blob/main/bitcraze_crazyflie_2/LICENSE

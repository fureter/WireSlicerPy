# WireSlicerPy
 
### Features:
- GUI interface
- Wing section design
- 4-Axis wirecutter machine customization
- Dynamic wire tensioning with auxilary motor
- CAD model decomposition into cross sections
- Layout planning for cutting complex 3D surfaces and recombining

Youtube playlist:
https://www.youtube.com/watch?v=KfWDov1mHDg&list=PLoByfIhR8AFkBicSiMon9fOJ0aAK4qdLv&pp=sAgC

##Limited Prototype Robot support via URDF and Pinocchio
- environment.yml  with required libraries for conda setup
- Examples STLs and URDF in assets folder
- Not all input arguments are currently supported
- Cutting tool is currently baked into the URDF -> ['wire'] link
- Faux g-code generation: -> Marlin M280 Servo commands -> stepper commands with G90 and R as the motor

Example usage:
  python robot_demo_6dof.py -stl assets\STLs\chuckmcnight\rook.stl -robot assets\Robot\xArm1S\xArm1S.urdf -out_dir debug
  

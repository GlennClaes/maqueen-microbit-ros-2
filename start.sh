Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Raster
      Enabled: true
      Reference Frame: <Fixed Frame>
      Cell Size: 0.5
      Plane Cell Count: 16
      Color: 160; 160; 164
      Alpha: 0.3
    - Class: rviz_default_plugins/Map
      Name: Kaart
      Enabled: true
      Topic:
        Value: /map
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
        History Policy: Keep Last
      Color Scheme: map
      Alpha: 0.85
      Draw Behind: true
    - Class: rviz_default_plugins/LaserScan
      Name: Scan (360 graden)
      Enabled: true
      Topic:
        Value: /scan
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        History Policy: Keep Last
      Style: Spheres
      Size (m): 0.025
      Color Transformer: FlatColor
      Color: 239; 41; 41
      Decay Time: 0
    - Class: rviz_default_plugins/Range
      Name: Ultrasoon
      Enabled: true
      Topic:
        Value: /maqueen/range
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        History Policy: Keep Last
      Color: 252; 175; 62
      Alpha: 0.5
    - Class: rviz_default_plugins/Path
      Name: Gereden pad
      Enabled: true
      Topic:
        Value: /maqueen/traject
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        History Policy: Keep Last
      Color: 52; 101; 255
      Line Style: Lines
    - Class: rviz_default_plugins/Path
      Name: Gepland pad
      Enabled: true
      Topic:
        Value: /maqueen/plan
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        History Policy: Keep Last
      Color: 255; 215; 0
      Line Style: Billboards
      Line Width: 0.02
    - Class: rviz_default_plugins/MarkerArray
      Name: Robot, thuis en objecten
      Enabled: true
      Topic:
        Value: /maqueen/markers
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        History Policy: Keep Last
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: false
      Marker Scale: 0.3
  Global Options:
    Fixed Frame: map
    Background Color: 40; 40; 44
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Measure
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Bovenaanzicht
      Scale: 120
      X: 0
      Y: 0
      Angle: 0
Window Geometry:
  Width: 1280
  Height: 800

CarlaUE4.exe /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30
CarlaUE4.exe /Game/Maps/Course4 -quality-level=Low -windowed -carla-server -benchmark -fps=30
CarlaUE4.exe /Game/Maps/Course4 -RenderOffScreen -windowed -carla-server -benchmark -fps=30
CarlaUE4.exe /Game/Maps/Course4 -quality-level=Epic -windowed -carla-server -benchmark -fps=30
CarlaUE4.exe -windowed -carla-server -benchmark -fps=30

CarlaUE4.exe /Game/Maps/Town02 -windowed -carla-server -benchmark -fps=60

python module_7.py


darknet detector test C:/src/darknet/cfg/coco.data C:/src/darknet/cfg/yolov7.cfg C:/src/darknet/yolov7.weights C:/Users/danie/Documents/Documents/CURSOS/Self-Driving_Cars_Specialization/CarlaSimulator/PythonClient/Course4FinalProject/_out/episode_3360/CameraRGB/000001.png



wsl --shutdown

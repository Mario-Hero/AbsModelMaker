# AbsModelMaker 三维图形生成试验台

上次更新：2023/07/26

分形是很美丽也很有趣的东西，我打算通过C++代码生成三维分形模型，然后导出到obj格式，用3d软件读取并渲染，所以我设计了这个程序，帮助我构建和保存模型，还能通过OpenGL预览模型。

<img src="./img/Screenshot 2023-07-13 150551.jpg" alt="Screenshot 2023-07-13 150551" width="30%"/><img src="./img/Screenshot 2023-07-13 151149.jpg" alt="Screenshot 2023-07-13 151149" width="30%"/><img src="./img/Screenshot 2023-07-26 182648.jpg" alt="Screenshot 2023-07-26 182648" width="30%"/>

# 环境配置  

编译器需要支持C++ 17和OpenMP

在Windows和Linux下需要配置好OpenGL的GLFW、glad、glm。OpenGL版本在3.3以上即可。

我自己尝试在Windows 11下用Visual Studio 2022编译运行成功，在Fedora 38上用g++编译运行成功。g++命令如下：

```sh
g++ -std=c++17 -o out ./AbstractModelMaker.cpp ./glad.c ./camera.h ./ABMath.h ./Generator.cpp ./Generator.h -lglfw3 -lGL -lm -lXrandr -lXi -lX11 -lpthread -ldl -lXinerama -lXcursor -fopenmp
```

如果只需要生成模型而不需要OpenGL显示，可以只使用ABMath.h文件，配置好glm和OpenMP即可。

# 特色功能

### 挤出 extrude()

可以沿曲线挤出平面图形，曲线可以通过参数方程或导函数提供。

### 三角化 triangulate()

可以把平面图形三角化，仅保证正确渲染。采用的是简单的隔点连线的方法。未采用Delaunay方法。

### 删除重复点 Model::deleteDuplicatePoints()

### 删除重复面 Model::deleteDuplicateFaces()

### 体积转网格（并行） Volume::toMeshParallel()

使用Marching Cubes 方法，用OpenMP并行地把体积转换成三角网格。

Marching Cubes 源码来自 [Matt's Webcorner]([Matt's Webcorner - Marching Cubes (stanford.edu)](https://graphics.stanford.edu/~mdfisher/MarchingCubes.html))

# 一些示例

Generator.cpp的Generator::init()函数中，包含了一些示例函数，名称为demoX()，可以注释掉并保留其中一个，运行之即可看到该函数的效果。



# 操作

滚轮调整相机距离。鼠标左键拖动旋转相机。鼠标右键拖动移动相机。

| 键盘按键 | 功能                   |
| -------- | ---------------------- |
| 2        | 使用面法线显示         |
| 3        | 使用点法线平滑显示     |
| 4        | 线框显示               |
| 5        | 着色显示               |
| i        | 输出模型的点和三角面数 |
| s        | 保存到obj文件          |
| Esc      | 退出                   |

# License

The MIT License (MIT)
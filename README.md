# AbsModelMaker 三维图形生成试验台

上次更新：2023/07/13

分形是很美丽也很有趣的东西，我打算通过C++代码生成三维分形模型，然后导出到obj格式，用3d软件读取并渲染，所以我设计了这个程序，帮助我构建和保存模型，还能通过OpenGL预览模型。

<img src="./img/Screenshot 2023-07-13 150551.jpg" alt="Screenshot 2023-07-13 150551" width="30%"/><img src="./img/Screenshot 2023-07-13 151149.jpg" alt="Screenshot 2023-07-13 151149" width="30%"/>

# 环境配置  

编译器需要支持C++ 17

在Windows和Linux下需要配置好OpenGL的GLFW、glad、glm。OpenGL版本在3.3以上即可。

我自己尝试在Windows 11下用Visual Studio 2022编译运行成功，在Fedora 38上用g++编译运行成功。g++命令如下：

```sh
g++ -std=c++17 -o out ./AbstractModelMaker.cpp ./glad.c ./camera.h ./ABMath.h ./Generator.cpp ./Generator.h -lglfw3 -lGL -lm -lXrandr -lXi -lX11 -lpthread -ldl -lXinerama -lXcursor
```

如果只需要生成模型而不需要OpenGL显示，可以只使用ABMath.h文件，配置好glm即可。

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
| s        | 保存到obj文件          |
| i        | 输出模型的点和三角面数 |
| Esc      | 退出                   |

# License

The MIT License (MIT)

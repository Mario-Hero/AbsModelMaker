#pragma once
#include <string>
#include <vector>

#include "ABMath.h"
using namespace ABMath;

class Generator {
public:
    const std::string savePath = "./test.obj";
    std::vector<float> ov;  // vertexs for opengl display
    World world;
    void init();
    Generator();
};

#include "Generator.h"

#include <glm/glm.hpp>
#include <random>
#include <time.h>
#include "ABMath.h"

// 螺旋的参数方程
vec3 spiralFunction(const float t) {
    const float a = 5.f;
    const float b = 1.f;
    return vec3(a * cos(t), a * sin(t), b * t);
}

// LorenzSystem 洛伦兹系统
void demo1(Model& m) {
    const std::vector<vec2> n = {vec2(0, 0), vec2(0.5, 0), vec2(0.5, 0.1), vec2(0, 0.1)};
    auto lorenzSystemA = [](const vec3 p) {
        return lorenzSystem(p);
    };
    m.extrudeDFun(lorenzSystemA, n, vec3(2, 2, 1), 0.01, 30);
    m.refine();
}

// 绘制一些分形图形
void demo2(World& world) {
    auto k = Model("new");
    k.makePyramid(10.f);
    for (int i = 0; i < 5; ++i) {
        world.moveModel(
            Model().iterateFace(k.face2FaceVec(), kochi3d, i).movePos(float(i) * vec3(15, 0, 0)).refine());
    }
}

// double-pendulum 双摆的末端的轨迹
void demo3(Model& m) {
    double o1 = AB_PI - 0.04, o2 = AB_PI + 0.04, w1 = 0, w2 = 0;
    const double dt = 0.01;
    std::default_random_engine e;
    std::normal_distribution<double> u(0, 1);  // 均值为0，标准差为1
    e.seed(time(0));
    const float l1 = 1;
    const float l2 = 1;
    const float m1 = 1;
    const float m2 = 1;

    for (double i = 0; i < 10; i += dt) {
        const auto res = doublePendulum(o1, o2, w1, w2, m1, m2, l1, l2);
        w1 += dt * res.first;
        w2 += dt * res.second;
        o1 += w1 * dt;
        o2 += w2 * dt;
        const float x1 = l1 * sin(o1);
        const float y1 = -l1 * cos(o1);
        // const vec3 p1(x1, y1 , i);
        const vec3 p2(x1 + l2 * sin(o2), y1 - l2 * cos(o2), i);
        // addPoint(p2);
        m.makeUVSphere(p2, 0.01, 6, 6);  // 用球体来代替点，方便导出
    }
    m.refine();
}

// 石墨烯
void demo4(Model& m) {
    const vec3 grey = vec3(0.5, 0.5, 0.5);
    const auto cirVec = circle(0.07, 10);
    const size_t ballSliceA = 10;
    const size_t ballSliceB = 8;
    const int L = 7;
    const float r = 1;
    const float ballR = 0.2;
    for (int i = 0; i < L; ++i) {
        vec3 lastPos;
        for (int j = 0; j < 6; ++j) {
            const vec3 pos = vec3(i * r * 1.5 + ((j + i) % 2) * r / 2, 0, j * r * 0.866);
            m.makeUVSphere(pos, ballR, ballSliceA, ballSliceB, grey);
            if (j != 0) {
                m.bar(cirVec, lastPos, pos, grey);
            }
            lastPos = pos;
            if ((j + i) % 2 == 0 && i != 0) {
                m.bar(cirVec, pos - vec3(r, 0, 0), pos, grey);
            }
        }
    }
    m.reCalNormalByFace();
}

// SpiralFunction 绘制螺旋
void demo5(Model& m) {
    const std::vector<vec2> n = {vec2(0, 0), vec2(1, 0), vec2(1, 1), vec2(0, 1)};
    m.extrude(spiralFunction, n, 0, 100, 0.1);  // 沿曲线挤出2D图形
    m.reCalNormalByFace();
}

// 新建多个重复正方体，展示删除重复点和面功能
void demo6(Model& m) {
    m.makeCube()
        .makeCube()
        .makeCube()
        .makeCube()
        .makeCube()
        .makeCube()
        .deleteDuplicatePoints()
        .deleteDuplicateFaces();
    m.reCalNormalByFace();
    m.showModelInfo();
}

// 曼德尔球的体积转网格
void demo7(Model& m) {
    const size_t RESOLUTION = 100;
    Volume vol({ RESOLUTION, RESOLUTION, RESOLUTION }, vec3(2.5, 2.5, 2.5));
    auto mandelbulbVec = [&](const vec3& p) {
        return mandelbulb(p.x, p.y, p.z);
    };
    std::cout << "making Volume" << std::endl;
    vol.makeVolume(mandelbulbVec);
    std::cout << "finish making" << std::endl;
    clock_t start, end;
    start = clock();
    m = vol.toMeshParallelNoDuplicatePoints();  //慢，但删除了所有重复顶点
    //m = vol.toMeshParallel();  //更快，但会保留很多重复的顶点
    end = clock();
    std::cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << std::endl;
    std::cout << "finish meshing" << std::endl;
    
}
Generator::Generator() {}

void Generator::init() {
    auto bal = Model("test");
    // 从下面的demo中挑一个执行即可
    // demo1(bal);
    // demo3(bal);
    // demo2(world);
    // demo4(bal);
    // demo5(bal);
    // demo6(bal);
    demo7(bal);
    // 下面的代码不需要调整
    world.moveModel(bal);
    ov = world.ovDisplay();
}

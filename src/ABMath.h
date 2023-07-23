#pragma once
#include <algorithm>
#include <array>
#include <fstream>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace ABMath {
using glm::mat4;
using glm::vec2;
using glm::vec3;
using glm::vec4;
constexpr vec3 WORLD_X = vec3(1, 0, 0);
constexpr vec3 WORLD_Y = vec3(0, 1, 0);
constexpr vec3 WORLD_Z = vec3(0, 0, 1);
constexpr vec3 WORLD_UP = WORLD_Y;
constexpr vec3 WORLD_ORIGIN = vec3(0, 0, 0);
constexpr vec3 NO_NORMAL = vec3(0, 0, 0);
constexpr vec3 DEFAULT_COLOR = vec3(1, 1, 1);
constexpr std::array<vec3, 6> WORLD_DIRECTION_ARRAY = {vec3(0, 0, 1), vec3(0, 0, -1), vec3(0, 1, 0), vec3(0, -1, 0), vec3(1, 0, 0), vec3(-1, 0, 0)};
constexpr double AB_PI = 3.141592653;
const std::string MODEL_NAME = "model";
const std::string WORLD_NAME = "world";
using PARAM_INPUT = vec3;  // {start, end, delta}

struct Point {
    vec3 pos;  // 位置
    vec3 nor;  // 法线Normal, don't need to normalize
    vec3 col;  // Color
    std::vector<float> sequence(const vec3& normal = NO_NORMAL) const {
        return normal == NO_NORMAL ? std::vector<float>({pos[0], pos[1], pos[2], nor[0], nor[1], nor[2], col[0], col[1], col[2]}) : std::vector<float>({pos[0], pos[1], pos[2], normal[0], normal[1], normal[2], col[0], col[1], col[2]});
    }
};

struct Face {  // 4边面，若Face[2]==Face[3],则为3边面
    std::array<int, 4> p;
    vec3 nor = NO_NORMAL;
    Face(const std::array<int, 4> arr = std::array<int, 4>(), const vec3 normal = NO_NORMAL) : p(arr), nor(normal) {}
};
using TriFace = std::array<int, 3>;  // 3边面
using Trail = std::vector<vec3>;     // 空间轨迹
using FaceVec = std::vector<vec3>;   // 带有点坐标的面

struct hash_vec3 {
    size_t operator()(const vec3& p) const {  // 别忘记const
        return std::hash<float>()(p[0]) + std::hash<float>()(p[1]) + std::hash<float>()(p[2]);
    }
};
inline float clamp(const float input, const float small, const float big) {
    if (input > big)
        return big;
    if (input > small)
        return input;
    return small;
}
inline float length(const float x, const float y, const float z) {
    return sqrt(x * x + y * y + z * z);
}
inline float smoothStep(const float input, const float xa, const float ya, const float xb, const float yb) {
    const float t = (clamp(input, xa, xb) - xa) / (xb - xa);
    const float f = t * t * (3 - 2 * t);
    return f * (yb - ya) + ya;
}
inline void outputVec3(const vec3 aPos) {
    std::cout << "Vec3" << std::endl;
    std::cout << aPos[0] << " " << aPos[1] << " " << aPos[2] << std::endl;
    std::cout << std::endl;
}
inline void outputMat4(const mat4 m) {
    std::cout << "Matrix4x4" << std::endl;
    std::cout << m[0][0] << " " << m[1][0] << " " << m[2][0] << " " << m[3][0] << std::endl;
    std::cout << m[0][1] << " " << m[1][1] << " " << m[2][1] << " " << m[3][1] << std::endl;
    std::cout << m[0][2] << " " << m[1][2] << " " << m[2][2] << " " << m[3][2] << std::endl;
    std::cout << m[0][3] << " " << m[1][3] << " " << m[2][3] << " " << m[3][3] << std::endl;
    std::cout << std::endl;
}
inline mat4 aLookAt(const vec3& eye, const vec3& front, const vec3& axisUp = WORLD_UP) {
    const vec3 fn = glm::normalize(front);
    const vec3 right = glm::normalize(glm::cross(fn, glm::normalize(axisUp)));
    const vec3 up = glm::normalize(glm::cross(right, fn));
    return mat4(vec4(right, 0), vec4(up, 0), vec4(-fn, 0), vec4(eye, 1));
}
inline float cross2d(const vec2& a, const vec2& b) {
    return a[0] * b[1] - a[1] * b[0];
}
// 获取参数方程曲线在某点的方向向量
inline vec3 direction(std::function<vec3(const float)> line, const float x, const bool GO_UP = true, const float delta = 0.0001f) {
    const vec3 a = line(x);
    const vec3 b = line(x + delta);
    return glm::normalize(GO_UP ? (b - a) : (a - b));
}
// 返回平面正多边形，可提高cut值模拟圆形
inline std::vector<vec2> circle(const float r, const int cut) {
    std::vector<vec2> t;
    const double phi = 2 * AB_PI / float(cut);
    for (int i = 0; i < cut; ++i) {  // 至少3cut
        t.emplace_back(r * cos(i * phi), r * sin(i * phi));
    }
    return t;
}
// 返回平面正方形
inline std::vector<vec2> square(const float a) {
    return circle(a / 2.f, 4);
}
// 返回平面长方形
inline std::vector<vec2> rect(const float a, const float b) {
    // a为长，对应x轴；b为高，对应y轴。
    return std::vector<vec2>({{0, 0}, {a, 0}, {a, b}, {0, b}});
}
// 返回洛伦兹系统某个点的导数
inline vec3 lorenzSystem(const vec3 p, const float a = 10, const float b = 8 / 3, const float r = 28) {
    return vec3(a * (p.y - p.x), r * p.x - p.y - p.x * p.z, p.x * p.y - b * p.z);
}
// 返回双摆对应角度和速度时的加速度，Theta1=Theta2=pi时摆朝向正上方
inline std::pair<double, double> doublePendulum(const double Theta1, const double Theta2, const double Omega1, const double Omega2, const double m1, const double m2, const double l1, const double l2) {
    const double g = 9.8;
    const double alpha1 = (-g * (2 * m1 + m2) * sin(Theta1) - g * m2 * sin(Theta1 - 2 * Theta2) - 2 * m2 * sin(Theta1 - Theta2) * (Omega2 * Omega2 * l2 + Omega1 * Omega1 * l1 * cos(Theta1 - Theta2))) / (l1 * (2 * m1 + m2 - m2 * cos(2 * Theta1 - 2 * Theta2)));
    const double alpha2 = (2 * sin(Theta1 - Theta2)) * (Omega1 * Omega1 * l1 * (m1 + m2) + g * (m1 + m2) * cos(Theta1) + Omega2 * Omega2 * l2 * m2 * cos(Theta1 - Theta2)) / l2 / (2 * m1 + m2 - m2 * cos(2 * Theta1 - 2 * Theta2));
    return std::make_pair(alpha1, alpha2);
}
// 曼德尔球
inline bool mandelbulb(const double c_x, const double c_y, const double c_z, const double power = 8, const int MAX_ITERATIONS = 10, const double BALLOUT = 2.0) {
    vec3 z = vec3(c_x, c_y, c_z);
    double r = 0.0;
    for (size_t i = 0; i < MAX_ITERATIONS; ++i) {
        r = sqrt(z.x * z.x + z.y * z.y + z.z * z.z);
        if (r > BALLOUT) {
            return false;  // should not paint this point
        }
        const double theta = std::acos(z.z / r) * power;
        const double phi = std::atan2(z.y, z.x) * power;
        const float zr = static_cast<float>(std::pow(r, power));
        z = vec3(c_x, c_y, c_z) + zr * vec3(std::sin(theta) * std::cos(phi), std::sin(phi) * std::sin(theta), std::cos(theta));
    }
    return true;
}
inline std::vector<FaceVec> kochi3d(const FaceVec& fv) {
    std::vector<FaceVec> t;
    const vec3& a = fv[0];
    const vec3& b = fv[1];
    const vec3& c = fv[2];
    const vec3 mab = (a + b) * 0.5f;
    const vec3 mbc = (c + b) * 0.5f;
    const vec3 mca = (a + c) * 0.5f;
    const vec3 m = (a + b + c) / 3.f;
    const vec3 n = glm::normalize(glm::cross(b - a, c - a)) * length(b - a) * 0.408248f;
    const vec3 q = m + n;
    t.emplace_back(FaceVec({a, mab, mca}));
    t.emplace_back(FaceVec({b, mbc, mab}));
    t.emplace_back(FaceVec({c, mca, mbc}));
    t.emplace_back(FaceVec({mab, mbc, q}));
    t.emplace_back(FaceVec({mbc, mca, q}));
    t.emplace_back(FaceVec({mca, mab, q}));
    return t;
}
// 三角化二维图形，返回由点的索引构成的三边面数组。使用非常简单的隔点连线的方法，仅保证正确渲染，不保证布线质量。
inline std::vector<TriFace> triangulate(const std::vector<vec2>& pv, const int indexStart = 0) {
    if (pv.size() <= 2)
        return std::vector<TriFace>();
    if (pv.size() == 3) {
        std::vector<TriFace> temp;
        const TriFace tf = {0, 1, 2};
        temp.emplace_back(tf);
        return temp;
    }
    std::vector<TriFace> out;
    std::vector<bool> sel;
    std::vector<float> convex;
    const int m = pv.size();
    sel.resize(m, false);
    convex.resize(m, 0);  //>0 凸 <0 凹 =0 共线

    auto calConvex = [&](const int i) -> float {
        const vec2 a = pv[i] - pv[(i + pv.size() - 1) % pv.size()];
        const vec2 b = pv[(i + 1) % pv.size()] - pv[i];
        return cross2d(a, b);
    };

    for (int i = 0; i < m; ++i) {
        convex[i] = calConvex(i);
    }

    auto calConvexWithSel = [&](const int i) -> float {
        int j = (i + 1) % pv.size();
        int k = (i - 1) % pv.size();
        while (sel[j]) {
            j = (j + 1) % pv.size();
        }
        while (sel[k]) {
            k = (k - 1) % pv.size();
        }
        const vec2 a = pv[i] - pv[k];
        const vec2 b = pv[j] - pv[i];
        return cross2d(a, b);
    };

    auto tryConnect = [&](const int ii) {
        int p = 0;
        int j = (ii + 1) % m;
        int j2 = 0;
        while (p < 2) {
            while (sel[j]) {
                j = (j + 1) % m;
            }
            ++p;
            if (p == 1) {
                j2 = j;
                j = (j + 1) % m;
            }
        }
        out.emplace_back(TriFace({ii + indexStart, j2 + indexStart, j + indexStart}));
        // std::cout << ii << " " << j2 << " " << j << std::endl;
        sel[j2] = true;
        convex[ii] = calConvexWithSel(ii);
        convex[j] = calConvexWithSel(j);
    };

    for (int i = 0; i < m; ++i) {
        if (convex[i] == 0) {
            sel[i] = true;
        }
    }
    bool allConvex = true;
    int i = 0;
    while (i < m) {
        while (sel[i]) {
            ++i;
            if (i == m)
                break;
        }
        if (i == m) {
            if (allConvex) {
                break;
            } else {
                allConvex = true;
                i = 0;
            }
        } else {
            if (convex[i] < 0) {
                tryConnect(i);
                allConvex = false;
            }
            ++i;
        }
    }
    i = 0;
    int selNum = 0;
    while (i < m) {
        while (sel[i] || convex[i] == 0) {
            ++selNum;
            ++i;
            if (i == m) {
                break;
            }
        }
        if (i == m) {
            if (pv.size() - selNum < 3) {
                break;
            } else {
                i = 0;
                selNum = 0;
            }
        } else {
            tryConnect(i);
            ++i;
        }
    }
    return out;
}

class Model {
private:
    std::mutex mtx;
    void calMiddleNormal(const int a, const int m, const int b)  // 点的顺序为 a->m->b
    {
        const vec3 va = p[a].pos - p[m].pos;
        const vec3 vb = p[b].pos - p[m].pos;
        p[m].nor += glm::normalize(glm::cross(vb, va));
    }
    void addPoint(const vec3 aPos, const vec3 aNormal = NO_NORMAL, const vec3 aColor = DEFAULT_COLOR) {
        p.emplace_back(Point{aPos, aNormal, aColor});
    }
    void addPoint(const float x, const float y, const float z) {
        p.emplace_back(Point{vec3(x, y, z), NO_NORMAL, DEFAULT_COLOR});
    }
    void addFace(const Face& ftemp) {
        f.emplace_back(ftemp);
    }
    void addFace(const int a, const int b, const int c, const int d, const vec3& normal = NO_NORMAL) {
        f.emplace_back(Face({a, b, c, d}, normal));
    }
    void addFaceInvert(const int a, const int b, const int c, const int d, const vec3& normal = NO_NORMAL) {
        f.emplace_back(Face({d, c, b, a}, normal));
    }

    void addTriFace(const TriFace& ftemp) {
        f.emplace_back(Face({ftemp[0], ftemp[1], ftemp[2], ftemp[2]}));
    }
    void addTriFace(const int a, const int b, const int c) {
        f.emplace_back(Face({a, b, c, c}));
    }

public:
    std::string name = "";
    std::vector<Point> p;
    std::vector<Face> f;
    Model(const std::string inputName = MODEL_NAME) : name(inputName){};
    Model(const Model& mo) {
        p = mo.p;
        f = mo.f;
        name = mo.name;
    }
    // lock the mutex of this model. Remember to unlock
    Model& lock() {
        mtx.lock();
        return *this;
    }
    Model& unlock() {
        mtx.unlock();
        return *this;
    }
    Model& reCalNormalByPoint()  // 重新计算当前模型的法线，逐点平均
    {
        for (const auto& fii : f) {
            const auto& fi = fii.p;
            if (fi[2] == fi[3]) {
                const vec3 v0 = p[fi[1]].pos - p[fi[0]].pos;
                const vec3 v2 = p[fi[0]].pos - p[fi[2]].pos;
                const vec3 nor = glm::normalize(glm::cross(v2, v0));  // 逆时针为正面 GL_CCW
                p[fi[0]].nor += nor;
                p[fi[1]].nor += nor;
                p[fi[2]].nor += nor;
            } else {
                calMiddleNormal(fi[0], fi[1], fi[2]);
                calMiddleNormal(fi[1], fi[2], fi[3]);
                calMiddleNormal(fi[2], fi[3], fi[0]);
                calMiddleNormal(fi[3], fi[0], fi[1]);
            }
        }
        return *this;
    }
    Model& reCalNormalByFace()  // 重新计算当前模型的法线，逐面计算
    {
        for (auto& fi : f) {
            const int a = fi.p[0];
            const int m = fi.p[1];
            const int b = fi.p[2];
            const vec3 va = p[a].pos - p[m].pos;
            const vec3 vb = p[b].pos - p[m].pos;
            fi.nor = glm::normalize(glm::cross(vb, va));
        }
        return *this;
    }
    Model& showModelInfo() {
        unsigned int triangleFaceNum = 0;
        for (const auto& fi : f) {
            triangleFaceNum += fi.p[2] == fi.p[3] ? 1 : 2;
        }
        std::cout << "Model: " << name << std::endl;
        std::cout << "Vertice: " << p.size() << std::endl;
        std::cout << "Triangles: " << triangleFaceNum << std::endl;
        std::cout << std::endl;
        return *this;
    }

    // 根据面的情况计算法线
    Model& refine() {
        if (p.empty() || f.empty())
            return *this;
        if (f[0].nor != NO_NORMAL) {
            // reCalNormalByFace();
        } else if (p[0].nor != NO_NORMAL) {
            reCalNormalByPoint();
        } else {
            reCalNormalByFace();
        }
        return *this;
    }
    Model& save2Obj(std::string savePath) {
        std::ofstream objOut(savePath, std::ios::ate);
        if (!objOut.is_open()) {
            std::cout << "Cannot save Model to " << savePath << std::endl;
            return *this;
        }
        std::cout << "Now saving Model: " << name << " to " << savePath << std::endl;
        objOut << "#AbstractModelMaker by Mario-Hero\n";
        objOut << "g " << name << "\n";
        for (const Point& pi : p) {
            objOut << "v " << pi.pos[0] << " " << pi.pos[1] << " " << pi.pos[2] << " " << pi.col[0] << " " << pi.col[1] << " " << pi.col[2] << "\n";
        }
        for (const auto& f : f) {
            objOut << "vn " << f.nor[0] << " " << f.nor[1] << " " << f.nor[2] << "\n";
        }
        // 带法向量的面
        for (int i = 0; i < f.size(); ++i) {
            const auto& fa = f[i];
            const auto& fi = fa.p;
            const auto& fn = fa.nor;
            if (fi[2] == fi[3])
                objOut << "f " << fi[0] + 1 << "//" << i + 1 << " " << fi[1] + 1 << "//" << i + 1 << " " << fi[2] + 1 << "//" << i + 1 << "\n";
            else
                objOut << "f " << fi[0] + 1 << "//" << i + 1 << " " << fi[1] + 1 << "//" << i + 1 << " " << fi[2] + 1 << "//" << i + 1 << " " << fi[3] + 1 << "//" << i + 1 << "\n";
        }
        objOut.close();
        std::cout << "Finish!" << std::endl;
        return *this;
    }
    std::vector<FaceVec> face2FaceVec() const {
        std::vector<FaceVec> fv;
        for (const Face& fa : f) {
            FaceVec temp;
            for (const int fap : fa.p) {
                if (fap >= p.size()) {
                    throw("face2FaceVec():fap bigger than p");
                }
                temp.emplace_back(p[fap].pos);
            }
            if (temp.size() < 3)
                throw("face2FaceVec(): temp.size()<3");
            fv.emplace_back(temp);
        }
        return fv;
    }

    // faceVec转换成Face并储存
    Model& faceVec2Face(const std::vector<FaceVec>& fv) {
        const size_t pIndex = p.size();
        std::vector<vec3> vVec;
        std::unordered_map<vec3, size_t, hash_vec3> map;
        for (size_t i = 0; i < pIndex; ++i) {
            if (map.find(p[i].pos) == map.cend()) {
                map.insert(std::make_pair(p[i].pos, i));
            }
        }
        size_t iv = pIndex;
        for (const auto& fa : fv) {
            Face temp;
            for (int i = 0; i < fa.size() && i < 4; ++i) {
                const vec3& fav = fa[i];
                auto it = map.find(fav);
                if (it == map.cend()) {
                    map.insert(std::make_pair(fav, iv));
                    temp.p[i] = iv;
                    addPoint(fav);
                    ++iv;
                } else {
                    temp.p[i] = it->second;
                }
            }
            if (fa.size() == 3) {
                temp.p[3] = temp.p[2];
            }
            addFace(temp);
        }
        return *this;
    }

    // 沿曲线挤出图形
    Model& extrude(std::function<vec3(const float)> line, const std::vector<vec2>& pVec, const float tStart, const float tEnd, const float tStep, const vec3& Color = DEFAULT_COLOR) {
        const int pvStartIndex = p.size();
        const std::vector<TriFace> tr = triangulate(pVec, pvStartIndex);
        const vec3 startPoint = line(tStart);
        const mat4 tranStart = aLookAt(startPoint, direction(line, tStart, tStart < tEnd), WORLD_UP);  // 获取旋转矩阵
        vec3 upVector = vec3(tranStart[1]);
        const int m = pVec.size();
        for (const auto& pvec2 : pVec) {
            const vec4 tempVec4 = tranStart * vec4(pvec2, 0, 1);
            addPoint(vec3(tempVec4), NO_NORMAL, Color);
        }
        for (const auto& tri : tr) {
            addTriFace(tri);
        }
        int ik = 1;
        float i = tStart + tStep;
        for (; (i < tEnd && tStart < tEnd) || (i > tEnd && tStart > tEnd); i += tStep) {
            const vec3 nowPoint = line(i);
            const mat4 tran = aLookAt(nowPoint, direction(line, i, tStart < tEnd), upVector);  // 获取旋转矩阵
            upVector = vec3(tran[1]);
            for (int j = 0; j < int(pVec.size()); ++j) {
                const vec4 np = tran * vec4(pVec[j], 0, 1);
                addPoint(vec3(np), NO_NORMAL, Color);
                addFace(pvStartIndex + ik * m + j, pvStartIndex + ik * m + (j + 1) % m, pvStartIndex + ik * m + (j + 1) % m - m, pvStartIndex + ik * m + j - m);
            }
            ++ik;
        }
        const int deltaM = (ik - 1) * m;
        for (const auto& tri : tr) {
            addTriFace(tri[0] + deltaM, tri[2] + deltaM, tri[1] + deltaM);
        }
        return *this;
    }

    // 根据曲线的导数和起点挤出图形
    Model& extrudeDFun(std::function<vec3(vec3)> dLine, const std::vector<vec2>& pVec, const vec3& initialPoint, const float delta, const float length, const vec3& Color = DEFAULT_COLOR) {
        const int pvStartIndex = p.size();
        const std::vector<TriFace> tr = triangulate(pVec, pvStartIndex);
        const mat4 tranStart = aLookAt(initialPoint, glm::normalize(dLine(initialPoint)), WORLD_UP);  // 获取旋转矩阵
        /*
        const vec3 FRONT = vec3(0, 0, -1);
        const vec3 RIGHT = vec3(1, 0, 0);
        const vec3 UP = vec3(0, 1, 0);
        */
        vec3 upVector = vec3(tranStart[1]);
        const int m = pVec.size();
        for (const auto& pvec2 : pVec) {
            const vec4 tempVec4 = tranStart * vec4(pvec2, 0, 1);
            addPoint(vec3(tempVec4), NO_NORMAL, Color);
        }
        for (const auto& tri : tr) {
            addTriFace(tri);
        }
        int ik = 1;
        float i = delta;
        vec3 v = initialPoint;
        for (; i < length; i += delta) {
            const vec3 dir3 = dLine(v);
            const vec3 lastUpVector = upVector;
            const mat4 tran = aLookAt(v, dir3, upVector);  // 获取旋转矩阵
            upVector = vec3(tran[1]);
            v += dir3 * delta;
            for (int j = 0; j < int(pVec.size()); ++j) {
                const vec4 np = tran * vec4(pVec[j], 0, 1);
                addPoint(vec3(np), NO_NORMAL, Color);
                addFace(pvStartIndex + ik * m + j, pvStartIndex + ik * m + (j + 1) % m, pvStartIndex + ik * m + (j + 1) % m - m, pvStartIndex + ik * m + j - m);
            }
            ++ik;
        }
        const int deltaM = (ik - 1) * m;
        for (const auto& tri : tr) {
            addTriFace(tri[0] + deltaM, tri[2] + deltaM, tri[1] + deltaM);
        }
        return *this;
    }
    Model& plane(const vec3& pos = WORLD_ORIGIN, const vec3& axisX = vec3(1, 0, 0), const vec3& axisZ = vec3(0, 0, 1), const float xLen = 1, const float zLen = 1, const size_t divX = 5, const size_t divZ = 5) {
        const int pvStartIndex = p.size();
        const float zDelta = zLen / float(divZ);
        const float xDelta = xLen / float(divX);
        const vec3 upVector = glm::normalize(glm::cross(axisZ, axisX));
        for (size_t i = 0; i < divX; ++i) {
            for (size_t j = 0; j < divZ; ++j) {
                addPoint(pos + axisZ * (j * zDelta) + axisX * (i * xDelta));
            }
        }
        for (size_t i = 0; i < divX - 1; ++i) {
            for (size_t j = 0; j < divZ - 1; ++j) {
                const int loc = pvStartIndex + i * divZ + j;
                addFace(loc, loc + 1, loc + 1 + divZ, loc + divZ, upVector);
            }
        }
        return *this;
    }
    // 绘制柱体
    Model& bar(const std::vector<vec2>& pVec, const vec3& startPoint, const vec3& endPoint, const vec3& Color) {
        const int pvStartIndex = p.size();
        const std::vector<TriFace> tr = triangulate(pVec, pvStartIndex);
        const vec3 dir = glm::normalize(endPoint - startPoint);
        const mat4 tranStart = aLookAt(startPoint, dir, WORLD_UP);  // 获取旋转矩阵
        vec3 upVector = vec3(tranStart[1]);
        const int m = pVec.size();
        for (const auto& pvec2 : pVec) {
            const vec4 tempVec4 = tranStart * vec4(pvec2, 0, 1);
            addPoint(vec3(tempVec4), NO_NORMAL, Color);
        }
        for (const auto& tri : tr) {
            addTriFace(tri);
        }
        const mat4 tran = aLookAt(endPoint, dir, upVector);  // 获取旋转矩阵
        // upVector = vec3(tran[1]);
        for (int j = 0; j < pVec.size(); ++j) {
            const vec4 np = tran * vec4(pVec[j], 0, 1);
            addPoint(vec3(np), NO_NORMAL, Color);
            addFace(pvStartIndex + m + j, pvStartIndex + m + (j + 1) % m, pvStartIndex + (j + 1) % m, pvStartIndex + j);
        }
        for (const auto& tri : tr) {
            addTriFace(tri[0] + m, tri[2] + m, tri[1] + m);
        }
        return *this;
    }
    Model& plotWithDFun(std::function<vec3(vec3)> dLine, const vec3& initialLoc, const float delta, const float maxLength) {
        vec3 v = initialLoc;
        double t = 0.0;
        addPoint(initialLoc);
        while (t < maxLength) {
            v += dLine(v) * delta;
            addPoint(v);
            t += delta;
        }
        return *this;
    }
    // 两端颜色不同的柱体，颜色在柱子的中间突变。
    Model& barChem(const std::vector<vec2>& pVec, const vec3& startPoint, const vec3& endPoint, const vec3& Color1, const vec3& Color2) {
        const size_t pvStartIndex = p.size();
        const std::vector<TriFace> tr = triangulate(pVec, pvStartIndex);
        const vec3 dir = glm::normalize(endPoint - startPoint);
        const mat4 tranStart = aLookAt(startPoint, dir, WORLD_UP);  // 获取旋转矩阵
        vec3 upVector = vec3(tranStart[1]);
        const int m = pVec.size();
        for (const auto& pvec2 : pVec) {
            const vec4 tempVec4 = tranStart * vec4(pvec2, 0, 1);
            addPoint(vec3(tempVec4), NO_NORMAL, Color1);
        }
        for (const auto& tri : tr) {
            addTriFace(tri);
        }
        int ik = 1;
        const mat4 tran1 = aLookAt(0.495f * (endPoint - startPoint) + startPoint, dir, upVector);  // 获取旋转矩阵
        for (int j = 0; j < pVec.size(); ++j) {
            const vec4 np = tran1 * vec4(pVec[j], 0, 1);
            addPoint(vec3(np), NO_NORMAL, Color1);
            addFace(pvStartIndex + ik * m + j, pvStartIndex + ik * m + (j + 1) % m, pvStartIndex + ik * m + (j + 1) % m - m, pvStartIndex + ik * m + j - m);
        }
        ++ik;
        const mat4 tran2 = aLookAt(0.505f * (endPoint - startPoint) + startPoint, dir, upVector);  // 获取旋转矩阵
        for (int j = 0; j < pVec.size(); ++j) {
            const vec4 np = tran2 * vec4(pVec[j], 0, 1);
            addPoint(vec3(np), NO_NORMAL, Color2);
            addFace(pvStartIndex + ik * m + j, pvStartIndex + ik * m + (j + 1) % m, pvStartIndex + ik * m + (j + 1) % m - m, pvStartIndex + ik * m + j - m);
        }
        ++ik;
        const mat4 tran = aLookAt(endPoint, dir, upVector);  // 获取旋转矩阵
        // upVector = vec3(tran[1]);
        for (int j = 0; j < pVec.size(); ++j) {
            const vec4 np = tran * vec4(pVec[j], 0, 1);
            addPoint(vec3(np), NO_NORMAL, Color2);
            addFace(pvStartIndex + ik * m + j, pvStartIndex + ik * m + (j + 1) % m, pvStartIndex + ik * m + (j + 1) % m - m, pvStartIndex + ik * m + j - m);
        }
        for (const auto& tri : tr) {
            addTriFace(tri[0] + ik * m, tri[2] + ik * m, tri[1] + ik * m);
        }
        return *this;
    }
    // 绘制UV球
    Model& makeUVSphere(const vec3 pos = WORLD_ORIGIN, const float R = 1, const int n_slices = 12, const int n_stacks = 8, const vec3& Color = DEFAULT_COLOR) {
        const int s = p.size();
        // add top vertex
        const vec3 v0 = vec3(0, R, 0) + pos;
        addPoint(v0, vec3(0, 1, 0), Color);
        // generate vertices per stack / slice
        for (int i = 0; i < n_stacks - 1; ++i) {
            const double phi = AB_PI * double(i + 1) / double(n_stacks);  // stack 从下往上，不包括两个极点
            for (int j = 0; j < n_slices; ++j) {
                const double theta = 2.0 * AB_PI * double(j) / double(n_slices);
                const float x = std::sin(phi) * std::cos(theta);
                const float y = std::cos(phi);
                const float z = std::sin(phi) * std::sin(theta);
                addPoint(R * vec3(x, y, z) + pos, vec3(x, y, z), Color);
            }
        }
        // add bottom vertex
        const vec3 v1 = vec3(0, -R, 0) + pos;
        const int s2 = p.size();
        addPoint(v1, vec3(0, -1, 0), Color);
        // add top / bottom triangles
        for (int i = 0; i < n_slices; ++i) {
            auto i0 = i + 1;
            auto i1 = (i + 1) % n_slices + 1;
            addTriFace(s, s + i1, s + i0);
            i0 = i + n_slices * (n_stacks - 2) + 1;
            i1 = (i + 1) % n_slices + n_slices * (n_stacks - 2) + 1;
            addTriFace(s2, s + i0, s + i1);
        }

        // add quads per stack / slice
        for (int j = 0; j < n_stacks - 2; j++) {
            const int j0 = j * n_slices + 1;
            const int j1 = (j + 1) * n_slices + 1;
            for (int i = 0; i < n_slices; i++) {
                const int i0 = j0 + i;
                const int i1 = j0 + (i + 1) % n_slices;
                const int i2 = j1 + (i + 1) % n_slices;
                const int i3 = j1 + i;
                addFace(s + i0, s + i1, s + i2, s + i3);
            }
        }
        return *this;
    }
    // 绘制立方体
    Model& makeCube(const vec3 pos = WORLD_ORIGIN, const float len = 1, const vec3& Color = DEFAULT_COLOR) {
        const float r = len / 2.f;
        const int s = p.size();
        // 正面4个点，右上角开始，逆时针旋转
        addPoint(pos + vec3(r, r, r), NO_NORMAL, Color);
        addPoint(pos + vec3(-r, r, r), NO_NORMAL, Color);
        addPoint(pos + vec3(-r, -r, r), NO_NORMAL, Color);
        addPoint(pos + vec3(r, -r, r), NO_NORMAL, Color);
        // 背面4个点，右上角开始，逆时针旋转
        addPoint(pos + vec3(r, r, -r), NO_NORMAL, Color);
        addPoint(pos + vec3(-r, r, -r), NO_NORMAL, Color);
        addPoint(pos + vec3(-r, -r, -r), NO_NORMAL, Color);
        addPoint(pos + vec3(r, -r, -r), NO_NORMAL, Color);
        addFace(s, s + 1, s + 2, s + 3);
        addFace(s + 4, s, s + 3, s + 7);
        addFace(s + 5, s + 4, s + 7, s + 6);
        addFace(s + 1, s + 5, s + 6, s + 2);
        addFace(s + 5, s + 1, s, s + 4);
        addFace(s + 7, s + 3, s + 2, s + 6);
        return *this;
    }
    // 绘制格式为f_xyz(u,v)的3d参数方程
    Model& drawParametricEquation(const vec3 pos = WORLD_ORIGIN, std::function<vec3(const float, const float)> paramEq = std::function<vec3(const float, const float)>(), const PARAM_INPUT uparam = vec3(0, 1, 0.1), const PARAM_INPUT vparam = vec3(0, 1, 0.1), const bool double_sided = true) {
        if (paramEq) {
            size_t uSize = 0;
            size_t vSize = 0;

            for (float u = uparam[0]; (u < uparam[1] && uparam[2] > 0) || (u > uparam[1] && uparam[2] < 0); u += uparam[2])
                ++uSize;
            for (float v = vparam[0]; (v < vparam[1] && vparam[2] > 0) || (v > vparam[1] && vparam[2] < 0); v += vparam[2])
                ++vSize;

            size_t loc = p.size();
            size_t ui = 0;
            size_t vi = 0;
            for (float v = vparam[0]; (v < vparam[1] && vparam[2] > 0) || (v > vparam[1] && vparam[2] < 0); v += vparam[2]) {
                ui = 0;
                for (float u = uparam[0]; (u < uparam[1] && uparam[2] > 0) || (u > uparam[1] && uparam[2] < 0); u += uparam[2]) {
                    addPoint(paramEq(u, v) + pos);
                    if (ui != 0 && ui != uSize - 1 && vi != 0 && vi != vSize - 1) {
                        addFace(loc, loc + uSize, loc + uSize - 1, loc - 1);
                        if (double_sided)
                            addFaceInvert(loc, loc + uSize, loc + uSize - 1, loc - 1);
                    }
                    ++loc;
                    ++ui;
                }
                ++vi;
            }
            return *this;
        }
    }

    Model& makeHyperbolicHelicoid(const vec3 pos = WORLD_ORIGIN, const PARAM_INPUT uparam = vec3(-1, 1, 0.05), const PARAM_INPUT vparam = vec3(0, 1, 0.05), const float tau = 4, const bool double_sided = true) {
        auto hyperbolicHelicoid = [tau](const float u, const float v) -> vec3 {
            const double d = 1.0 / (1.0 + std::cosh(u) * std::cosh(v));
            return vec3(std::sinh(v) * std::cos(tau * u) * d, std::sinh(v) * std::sin(tau * u) * d, std::cosh(v) * std::sinh(u) * d);
        };

        drawParametricEquation(pos, hyperbolicHelicoid, uparam, vparam, double_sided);
        return *this;
    }
    // 绘制正3棱锥，a为边长
    Model& makePyramid(const float a = 1.f) {
        const int s = p.size();
        addPoint(a / 2, 0, a / 3.4641);
        addPoint(0, 0, -a / 1.732);
        addPoint(-a / 2, 0, a / 3.4641);
        addPoint(0, 0.816496 * a, 0);
        addTriFace(s + 2, s + 1, s);
        addTriFace(s, s + 1, s + 3);
        addTriFace(s + 1, s + 2, s + 3);
        addTriFace(s + 2, s, s + 3);
        return *this;
    }
    // 用一个函数迭代模型
    Model& iterateFace(const std::vector<FaceVec>& initialFaces, std::function<std::vector<FaceVec>(FaceVec)> iterator, const size_t times) {
        std::vector<FaceVec> fv = initialFaces;
        for (size_t i = 0; i < times; ++i) {
            std::vector<FaceVec> tempF;
            for (const auto& fa : fv) {
                const std::vector<FaceVec> res = iterator(fa);
                tempF.insert(tempF.end(), res.begin(), res.end());
            }
            fv = tempF;
        }
        faceVec2Face(fv);
        return *this;
    }
    // 移动模型
    Model& movePos(const vec3& pos) {
        for (auto& pp : p) {
            pp.pos += pos;
        }
        return *this;
    }
};
class World {
public:
    std::vector<Model> m;
    bool pointCloudMode = false;
    bool usePointNormal = false;
    bool exportObjAsDifferentObject = true;
    void refine() {
        for (auto& mo : m) {
            mo.refine();
        }
    }
    void updateNormals() {
        if (usePointNormal) {
            for (auto& mo : m) {
                mo.reCalNormalByPoint();
            }
        } else {
            for (auto& mo : m) {
                mo.reCalNormalByFace();
            }
        }
    }
    void moveModel(Model& mi) {
        m.emplace_back(std::move(mi));
    }
    void copyModel(const Model mi) {
        m.emplace_back(mi);
    }
    void saveWorld2Obj(const std::string savePath) const {
        std::ofstream objOut(savePath, std::ios::ate);
        if (!objOut.is_open()) {
            std::cout << "Cannot save Model to " << savePath << std::endl;
            return;
        }
        std::cout << "Now saving World to " << savePath << std::endl;
        objOut << "#AbstractModelMaker by Mario-Hero\n";
        if (!exportObjAsDifferentObject) {
            objOut << "o " << WORLD_NAME << "\n";
        }
        size_t pLoc = 1;
        for (const auto& mo : m) {
            objOut << (exportObjAsDifferentObject ? "o " : "g ") << mo.name << "\n";
            for (const Point& pi : mo.p) {
                objOut << "v " << pi.pos[0] << " " << pi.pos[1] << " " << pi.pos[2] << " " << pi.col[0] << " " << pi.col[1] << " " << pi.col[2] << "\n";
            }
            for (const auto& f : mo.f) {
                objOut << "vn " << f.nor[0] << " " << f.nor[1] << " " << f.nor[2] << "\n";
            }
            // 带法向量的面
            for (int i = 0; i < mo.f.size(); ++i) {
                const auto& fa = mo.f[i];
                const auto& fi = fa.p;
                const auto& fn = fa.nor;
                if (fi[2] == fi[3])
                    objOut << "f " << fi[0] + pLoc << "//" << i + pLoc << " " << fi[1] + pLoc << "//" << i + pLoc << " " << fi[2] + pLoc << "//" << i + pLoc << "\n";
                else
                    objOut << "f " << fi[0] + pLoc << "//" << i + pLoc << " " << fi[1] + pLoc << "//" << i + pLoc << " " << fi[2] + pLoc << "//" << i + pLoc << " " << fi[3] + pLoc << "//" << i + pLoc << "\n";
            }
            pLoc += mo.p.size();
        }
        objOut.close();
        std::cout << "Finish!" << std::endl;
    }
    std::vector<float> ovDisplay() const {
        std::vector<float> ov;
        auto ovAddPoint = [&](const int a, const int mIndex, const vec3& normal) {
            const auto seq = m[mIndex].p[a].sequence(normal);
            ov.insert(ov.end(), seq.begin(), seq.end());
        };
        auto displayTriFace = [&](const int a, const int b, const int c, const int mIndex, const vec3& normal) {
            ovAddPoint(a, mIndex, normal);
            ovAddPoint(b, mIndex, normal);
            ovAddPoint(c, mIndex, normal);
        };
        if (pointCloudMode) {
            for (const auto& mo : m) {
                for (const auto& p : mo.p) {
                    const auto seq = p.sequence(vec3(0, 0, -1));
                    ov.insert(ov.end(), seq.begin(), seq.end());
                }
            }
        } else {
            for (int i = 0; i < m.size(); ++i) {
                for (const auto& f : m[i].f) {
                    const auto& fi = f.p;
                    const vec3& normal = usePointNormal ? NO_NORMAL : f.nor;
                    displayTriFace(fi[0], fi[1], fi[2], i, normal);
                    if (fi[2] != fi[3]) {
                        displayTriFace(fi[0], fi[2], fi[3], i, normal);
                    }
                }
            }
        }
        return ov;
    }
    void saveWorld2ObjWithoutNormals(const std::string savePath) const {
        std::ofstream objOut(savePath, std::ios::ate);
        if (!objOut.is_open())
            return;
        std::cout << "Now saving to obj without normals..." << std::endl;
        objOut << "#AbstractModelMaker by Mario-Hero\n";
        if (!exportObjAsDifferentObject) {
            objOut << "o " << WORLD_NAME << "\n";
        }
        for (const auto& mo : m) {
            objOut << (exportObjAsDifferentObject ? "o " : "g ") << mo.name << "\n";
            for (const Point& pi : mo.p) {
                objOut << "v " << pi.pos[0] << " " << pi.pos[1] << " " << pi.pos[2] << " " << pi.col[0] << " " << pi.col[1] << " " << pi.col[2] << "\n";
            }
            for (const auto& f : mo.f) {
                const auto& fi = f.p;
                if (fi[2] == fi[3])
                    objOut << "f " << fi[0] + 1 << " " << fi[1] + 1 << " " << fi[2] + 1 << "\n";
                else
                    objOut << "f " << fi[0] + 1 << " " << fi[1] + 1 << " " << fi[2] + 1 << " " << fi[3] + 1 << "\n";
            }
        }
        objOut.close();
        std::cout << "Finish!" << std::endl;
    }
    void saveWorld2ObjWithManySameNormals(const std::string savePath) const {
        std::ofstream objOut(savePath, std::ios::ate);
        if (!objOut.is_open())
            return;
        std::cout << "Now saving to obj with many same normals..." << std::endl;
        objOut << "#AbstractModelMaker by Mario-Hero\n";
        if (!exportObjAsDifferentObject) {
            objOut << "o " << WORLD_NAME << "\n";
        }
        for (auto& mo : m) {
            objOut << (exportObjAsDifferentObject ? "o " : "g ") << mo.name << "\n";
            for (const Point& pi : mo.p) {
                objOut << "v " << pi.pos[0] << " " << pi.pos[1] << " " << pi.pos[2] << " " << pi.col[0] << " " << pi.col[1] << " " << pi.col[2] << "\n";
            }
            std::vector<vec3> mv;
            std::vector<size_t> fIndex;
            for (auto& f : mo.f) {
                const auto it = std::find(mv.cbegin(), mv.cend(), f.nor);
                if (it == mv.cend()) {
                    mv.emplace_back(f.nor);
                    fIndex.emplace_back(mv.size() - 1);  // 用x来表示索引
                    objOut << "vn " << f.nor[0] << " " << f.nor[1] << " " << f.nor[2] << "\n";
                } else {
                    fIndex.emplace_back(it - mv.cbegin());
                }
            }
            // 带法向量的面
            for (int i = 0; i < mo.f.size(); ++i) {
                const auto& f = mo.f[i];
                const auto& fi = f.p;
                const int fn = fIndex[i];
                if (fi[2] == fi[3])
                    objOut << "f " << fi[0] + 1 << "//" << fn + 1 << " " << fi[1] + 1 << "//" << fn + 1 << " " << fi[2] + 1 << "//" << fn + 1 << "\n";
                else
                    objOut << "f " << fi[0] + 1 << "//" << fn + 1 << " " << fi[1] + 1 << "//" << fn + 1 << " " << fi[2] + 1 << "//" << fn + 1 << " " << fi[3] + 1 << "//" << fn + 1 << "\n";
            }
        }
        objOut.close();
        std::cout << "Finish!" << std::endl;
    }
};

}  // namespace ABMath

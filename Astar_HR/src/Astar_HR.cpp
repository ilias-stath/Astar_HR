#include <iostream>
#include "Astar_HR.h"


using namespace std;

// Astar_HR::Astar_HR(unsigned short xs, uint8_t ys, unsigned short xg, uint8_t yg, unsigned short xgOffset, uint8_t ygOffset, uint8_t xMin, uint8_t yMin, unsigned short xMax, uint8_t yMax, unsigned short* ob, unsigned short lenObs, bool bezier, uint8_t step) {
Astar_HR::Astar_HR(unsigned short xs, uint8_t ys, unsigned short xg, uint8_t yg, uint8_t xMin, uint8_t yMin, unsigned short xMax, uint8_t yMax, unsigned short* ob, unsigned short lenObs, bool bezier, uint8_t step) {
    this->xs = xs;
    this->ys = ys;
    this->xg = xg;
    this->yg = yg;
    // this->xgOffset = xgOffset;
    // this->ygOffset = ygOffset;
    this->xMin = xMin;
    this->yMin = yMin;
    this->xMax = xMax;
    this->yMax = yMax;
    this->lenPath = 0;
    this->nodeQty = 0;
    this->xPoints = nullptr;
    this->yPoints = nullptr;
    this->lenObs = lenObs;
    this->obstacles = (unsigned short*)malloc(sizeof(unsigned short) * this->lenObs);
    this->bezier = bezier;
    this->step = step;

    memcpy(obstacles, ob, this->lenObs * sizeof(unsigned short));
}

float Astar_HR::heuristic(unsigned short xn, uint8_t yn, unsigned short xg, uint8_t yg) {
    float r = sqrt(pow(xg - xn, 2) + pow(yg - yn, 2));
    return r;
}

double Astar_HR::binomial(unsigned short n, unsigned short k) {
    double res = 1;
    if (k > n - k) k = n - k;
    for (unsigned short i = 0; i < k; i++) {
        res *= (n - i);
        res /= (i + 1);
    }
    return res;
}


std::vector<Astar_HR::Vec2> Astar_HR::bezierPath(const std::vector<Astar_HR::Vec2>& controlPoints, unsigned short subdivisions) {
    std::vector<Astar_HR::Vec2> curve;
    unsigned short n = controlPoints.size() - 1;
    for (unsigned short i = 0; i <= subdivisions; i++) {
        float t = i / static_cast<float>(subdivisions);
        Astar_HR::Vec2 point(0, 0);
        for (unsigned short j = 0; j <= n; j++) {
            float coeff = binomial(n, j) * pow(t, j) * pow(1 - t, n - j);
            point += controlPoints[j] * coeff;
        }
        curve.push_back(point);
    }
    return curve;
}

float Astar_HR::distanceVec(const Vec2& a, const Vec2& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

Astar_HR::Vec2 Astar_HR::interpolateVec(const Vec2& a, const Vec2& b, float ratio) {
    return Vec2(
        a.x + ratio * (b.x - a.x),
        a.y + ratio * (b.y - a.y)
    );
}

std::vector<Astar_HR::Vec2> Astar_HR::resamplePath(
    const std::vector<Vec2>& inputPath,
    float spacing)
{
    std::vector<Vec2> output;

    if (inputPath.size() < 2 || spacing <= 0)
        return output;

    output.push_back(inputPath.front());

    float accumulatedDist = 0.0f;
    Vec2 previous = inputPath.front();

    for (size_t i = 1; i < inputPath.size(); i++) {

        Vec2 current = inputPath[i];
        float segmentLength = distanceVec(previous, current);

        while (accumulatedDist + segmentLength >= spacing) {

            float remaining = spacing - accumulatedDist;
            float ratio = remaining / segmentLength;

            Vec2 newPoint = interpolateVec(previous, current, ratio);
            output.push_back(newPoint);

            previous = newPoint;
            segmentLength = distanceVec(previous, current);
            accumulatedDist = 0.0f;
        }

        accumulatedDist += segmentLength;
        previous = current;
    }

    if (distanceVec(output.back(), inputPath.back()) > 1e-5f)
        output.push_back(inputPath.back());

    return output;
}

float Astar_HR::perpendicularDistance(
    const Vec2& pt,
    const Vec2& lineStart,
    const Vec2& lineEnd)
{
    float dx = lineEnd.x - lineStart.x;
    float dy = lineEnd.y - lineStart.y;

    if (dx == 0 && dy == 0)
        return distanceVec(pt, lineStart);

    float t = ((pt.x - lineStart.x) * dx +
        (pt.y - lineStart.y) * dy) /
        (dx * dx + dy * dy);

    float projX = lineStart.x + t * dx;
    float projY = lineStart.y + t * dy;

    return sqrt((pt.x - projX) * (pt.x - projX) +
        (pt.y - projY) * (pt.y - projY));
}

void Astar_HR::rdpRecursive(
    const std::vector<Vec2>& path,
    size_t start,
    size_t end,
    float epsilon,
    std::vector<bool>& keep)
{
    if (end <= start + 1)
        return;

    float maxDist = 0.0f;
    size_t index = start;

    for (size_t i = start + 1; i < end; i++) {
        float dist = perpendicularDistance(path[i], path[start], path[end]);

        if (dist > maxDist) {
            maxDist = dist;
            index = i;
        }
    }

    if (maxDist > epsilon) {
        keep[index] = true;

        rdpRecursive(path, start, index, epsilon, keep);
        rdpRecursive(path, index, end, epsilon, keep);
    }
}

std::vector<Astar_HR::Vec2> Astar_HR::simplifyRDP(const std::vector<Vec2>& path,float epsilon)
{
    if (path.size() < 2)
        return path;

    std::vector<bool> keep(path.size(), false);

    keep.front() = true;
    keep.back() = true;

    rdpRecursive(path, 0, path.size() - 1, epsilon, keep);

    std::vector<Vec2> result;

    for (size_t i = 0; i < path.size(); i++) {
        if (keep[i])
            result.push_back(path[i]);
    }

    return result;
}


void Astar_HR::expandNode(Node* dad) {
    Node son(0, 0, 0, 0, nullptr);
    short x = 0;
    short y = 0;
    bool obFlag = false;

    // Checking the points that are 1 move from the current point. 8 points in total
    // Starting from the top left and ending in the bottom right, moving left to right
    for (int8_t i = -1; i <= 1; i++) {
        x = dad->getX() + i * step;
        if (x >= xMin && x <= xMax) {
            for (int8_t j = -1; j <= 1; j++) {
                y = dad->getY() + j * step;
                if (i != 0 || j != 0) {
                    if (y >= yMin && y <= yMax) {
                        for (unsigned short k = 0; k < this->lenObs; k += 4) {
                            if ((x >= obstacles[k] && x <= obstacles[k + 2]) && (y >= obstacles[k + 1] && y <= obstacles[k + 3])) {
                                // cout << "In obstacle ->  xbl:" << obstacles[k] << " ,  ybl:" << obstacles[k+1] << " ,  xtr:" << obstacles[k+2] << " ,  ytr:" << obstacles[k+3] << " . Location is -> X:" << x << " ,  Y:" << y << endl;
                                obFlag = true;
                            }
                        }

                        if (!obFlag) {
                            if (!(visited.find({ x, y }) != visited.end())) {
                                //To set the diagonals as other cost(not good, do not use)
                                if (abs(i) == 1 && abs(j) == 1) {
                                    son.setParameters(x, y, dad->getTotalCost() - heuristic(dad->getX(), dad->getY(), this->xg, this->yg) + sqrt(2) * step, heuristic(x, y, this->xg, this->yg), dad);
                                }
                                else {
                                    son.setParameters(x, y, dad->getTotalCost() - heuristic(dad->getX(), dad->getY(), this->xg, this->yg) + 1 * step, heuristic(x, y, this->xg, this->yg), dad);
                                }
                                pq.push(son);
                                nodeQty++;
                            }
                        }
                        else {
                            obFlag = false;
                        }
                    }
                }
                y = y - j * step;
            }
        }
        x = x - i * step;
    }
}


void Astar_HR::setPath(Node* n) {
    std::priority_queue<Node, std::vector<Node>, CompareCost> empty_pq;
    pq.swap(empty_pq);

    // Node* nFinal = new Node(xgOffset, ygOffset, 0, 0, nullptr);

    // cout << n->getX() << "  ,  " << n->getY() << endl;

    // path.push(nFinal);
    // lenPath++;

    do {
        path.push(n);
        n = n->getParent();
        lenPath++;

    } while (n->getParent() != nullptr);

    path.push(n);
    lenPath++;

    visited.clear();
    std::vector<Vec2> controlPoints;

    if (bezier) {
        for (unsigned short i = 0; i < lenPath; i++) {
            n = path.top();
            path.pop();

            controlPoints.push_back(Vec2(n->getX(), n->getY()));
        }
        this->subdivisions = lenPath;

        std::vector<Vec2> bezierCurve = bezierPath(controlPoints, subdivisions);
        controlPoints.clear();
        std::vector<Vec2>().swap(controlPoints);

        /* Uniform Resampling */
        std::vector<Vec2> uniformCurve = resamplePath(bezierCurve, step);
        bezierCurve.clear();
        std::vector<Vec2>().swap(bezierCurve);

        /* RDP Simplification */
        float epsilon = step * 0.3;   // tunable safety parameter
        std::vector<Vec2> simplifiedCurve = simplifyRDP(uniformCurve, epsilon);
        uniformCurve.clear();
        std::vector<Vec2>().swap(uniformCurve);


        lenPath = simplifiedCurve.size();

        /* Allocate memory */
        xPoints = (float*)malloc(sizeof(float) * lenPath);
        yPoints = (float*)malloc(sizeof(float) * lenPath);

        if (xPoints == nullptr || yPoints == nullptr) {
            exit(1);
        }

        /* Copy data */
        unsigned short i = 0;
        for (const auto& point : simplifiedCurve) {
            xPoints[i] = point.x;
            yPoints[i] = point.y;
            i++;
        }
        simplifiedCurve.clear();
        std::vector<Vec2>().swap(simplifiedCurve);
    }
    else {
        // Allocate new memory for x and y
        xPoints = (float*)malloc(sizeof(float) * lenPath);
        yPoints = (float*)malloc(sizeof(float) * lenPath);

        if (xPoints == nullptr || yPoints == nullptr) {
            exit(1);
        }

        for (unsigned short i = 0; i < lenPath; i++) {
            n = path.top();
            path.pop();

            xPoints[i] = n->getX();
            yPoints[i] = n->getY();

        }
    }

}

void Astar_HR::getPath(float* x, float* y) {
    memcpy(x, xPoints, lenPath * sizeof(float));
    memcpy(y, yPoints, lenPath * sizeof(float));
    free(xPoints);
    free(yPoints);
    xPoints = nullptr;
    yPoints = nullptr;
}

unsigned short Astar_HR::pathGeneration() {
    Node* n = new Node(xs, ys, 0, heuristic(xs, ys, xg, yg), nullptr);

    while (true) {
        // n->printNode();
        if (n->getX() == xg && n->getY() == yg) {
            break;
        }

        if (!(visited.find({ n->getX(), n->getY() }) != visited.end())) {
            visited.insert({ n->getX(), n->getY() });
            expandNode(n);

        }
        else {
            delete n;
            n = nullptr;
            nodeQty--;
        }

        if (pq.empty()) {
            return -2;
        }

        n = new Node(pq.top());
        pq.pop();

    }
    delete obstacles;
    obstacles = nullptr;

    setPath(n);


    return lenPath;
}

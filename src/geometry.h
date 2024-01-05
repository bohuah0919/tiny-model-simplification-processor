#pragma once
#include <memory>
#include <iostream>
#include <igl/opengl/glfw/Viewer.h>

struct Relation {
    std::vector<int> neighbors;
    std::vector<int> planes;
    Relation() {
        neighbors = std::vector<int>();
        planes = std::vector<int>();
    }
};

struct NewVertexInfo {
    Eigen::VectorXd newVertex;
    double QEM;
};

struct Edge
{
    int firstIndex;
    int secondIndex;
    std::shared_ptr<Eigen::MatrixXd> vPtr;
    std::shared_ptr<std::vector<Relation>> relationPtr;
    std::shared_ptr<std::vector<Eigen::MatrixXd>> facesQPtr;
    NewVertexInfo newVertexInfo;

    Edge(int first, int second,
        std::shared_ptr<Eigen::MatrixXd> vp,
        std::shared_ptr<std::vector<Relation>> rp,
        std::shared_ptr<std::vector<Eigen::MatrixXd>> fp);

    bool operator==(const Edge& other) const;
    void updateNewVertexInfo();

    struct EdgeHash {
        std::size_t operator()(const Edge& e) const {
            return std::hash<int>()(e.firstIndex) ^ (std::hash<int>()(e.secondIndex) << 1);;
        }
    };

    struct EdgeComparator {
        bool operator()(const Edge& a, const Edge& b) const {
            if (a.newVertexInfo.QEM == b.newVertexInfo.QEM) {
                if (a.firstIndex == b.firstIndex) {
                    return a.secondIndex < b.secondIndex;
                }
                else {
                    return a.firstIndex < b.firstIndex;
                }
            }

            return a.newVertexInfo.QEM < b.newVertexInfo.QEM;
        }
    };

};

#include "solver.h"
SimplificationSolver::SimplificationSolver() {
    vertices = Eigen::MatrixXd::Zero(1, 3);
    faces = Eigen::MatrixXi::Zero(1, 3);
    outVertices = Eigen::MatrixXd::Zero(1, 3);
    outFaces = Eigen::MatrixXi::Zero(1, 3);
    auto relationGraph = std::vector<Relation>(vertices.rows(), Relation());
    auto facesQ = std::vector<Eigen::MatrixXd>(faces.rows(), Eigen::MatrixXd::Zero(4, 4));

    vaildVertices = std::vector<int>(vertices.rows(), 1);
    vPtr = std::make_shared<Eigen::MatrixXd>(vertices);
    fPtr = std::make_shared<Eigen::MatrixXi>(faces);
    relationPtr = std::make_shared<std::vector<Relation>>(relationGraph);
    facesQPtr = std::make_shared<std::vector<Eigen::MatrixXd>>(facesQ);
    records = std::vector<ContractRecord>();
    hasBoundary = findBoundary();
    edges = std::set<Edge, Edge::EdgeComparator>();

    prepare();
    computeVaildEdges();

    initialEdges = edges.size();
    startIterID = 0;
};

SimplificationSolver::SimplificationSolver(Eigen::MatrixXd V, Eigen::MatrixXi F) : vertices(V), faces(F), outVertices(V), outFaces(F) {
    auto relationGraph = std::vector<Relation>(vertices.rows(), Relation());
    auto facesQ = std::vector<Eigen::MatrixXd>(faces.rows(), Eigen::MatrixXd::Zero(4, 4));

    vaildVertices = std::vector<int>(vertices.rows(), 1);
    vPtr = std::make_shared<Eigen::MatrixXd>(vertices);
    fPtr = std::make_shared<Eigen::MatrixXi>(faces);
    relationPtr = std::make_shared<std::vector<Relation>>(relationGraph);
    facesQPtr = std::make_shared<std::vector<Eigen::MatrixXd>>(facesQ);
    records = std::vector<ContractRecord>();
    hasBoundary = findBoundary();
    edges = std::set<Edge, Edge::EdgeComparator>();

    prepare();
    computeVaildEdges();

    initialEdges = edges.size();
    startIterID = 0;
};


inline void SimplificationSolver::backTrace(int recordID) {
    ContractRecord cr = records[recordID];

    outVertices.row(cr.removeIndex) = cr.oldValue2;
    outVertices.row(cr.updateIndex) = cr.oldValue1;

    vaildVertices[cr.removeIndex] = 1;

    for (auto faceID : cr.updatedFacesIndexSet) {
        outFaces(faceID[0], faceID[1]) = cr.removeIndex;
    }
};

inline void SimplificationSolver::frontIterate(int recordID) {
    ContractRecord cr = records[recordID];

    outVertices.row(cr.removeIndex) = cr.newValue;
    outVertices.row(cr.updateIndex) = cr.newValue;

    vaildVertices[cr.removeIndex] = 0;

    for (auto faceID : cr.updatedFacesIndexSet) {
        outFaces(faceID[0], faceID[1]) = cr.updateIndex;
    }
};

void SimplificationSolver::setIterationState(int targetIterID, Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
    if (targetIterID < startIterID) {
        for (int i = 0; i < startIterID - targetIterID; i++) {
            int recordID = startIterID - i;
            backTrace(recordID);
        }
        startIterID = targetIterID;
    }
    else {
        for (int i = 0; i < targetIterID - startIterID; i++) {
            int recordID = startIterID + i;
            frontIterate(recordID);
        }
        startIterID = targetIterID;
    }

    std::vector<int> remainedFaces = std::vector<int>();
    for (int f = 0; f < outFaces.rows(); ++f) {
        if (!(outFaces(f, 0) == outFaces(f, 1) || outFaces(f, 0) == outFaces(f, 2) || outFaces(f, 1) == outFaces(f, 2))) {
            remainedFaces.push_back(f);
        }
    }
    Eigen::MatrixXi newF = Eigen::MatrixXi::Zero(remainedFaces.size(), 3);
    for (int i = 0; i < remainedFaces.size(); ++i) {
        newF.row(i) = outFaces.row(remainedFaces[i]);
    }

    V = outVertices;
    F = newF;
}

void SimplificationSolver::getCleanedData(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
    std::vector<int> remainedFaces = std::vector<int>();
    for (int f = 0; f < outFaces.rows(); ++f) {
        if (!(outFaces(f, 0) == outFaces(f, 1) || outFaces(f, 0) == outFaces(f, 2) || outFaces(f, 1) == outFaces(f, 2))) {
            remainedFaces.push_back(f);
        }
    }

    Eigen::MatrixXi newF = Eigen::MatrixXi::Zero(remainedFaces.size(), 3);
    for (int i = 0; i < remainedFaces.size(); ++i) {
        newF.row(i) = outFaces.row(remainedFaces[i]);
    }

    std::vector<int> verticesIndexMap = std::vector<int>(outVertices.rows(), 0);

    std::vector<int> remainedVertices = std::vector<int>();
    for (int i = 0; i < vaildVertices.size(); i++) {
        if (vaildVertices[i] == 1) {
            remainedVertices.push_back(i);
            verticesIndexMap[i] = remainedVertices.size() - 1;
        }
        else {
            verticesIndexMap[i] = -1;
        }
    }

    Eigen::MatrixXd newV = Eigen::MatrixXd::Zero(remainedVertices.size(), 3);
    for (int i = 0; i < remainedVertices.size(); ++i) {
        newV.row(i) = outVertices.row(remainedVertices[i]);
    }

    for (int f = 0; f < newF.rows(); ++f) {
        newF(f, 0) = verticesIndexMap[newF(f, 0)];
        newF(f, 1) = verticesIndexMap[newF(f, 1)];
        newF(f, 2) = verticesIndexMap[newF(f, 2)];
    }

    V = newV;
    F = newF;
}


inline void SimplificationSolver::updatePlaneInfo(int planeID) {
    Eigen::Vector3d e1 = (*vPtr).row((*fPtr)(planeID, 1)) - (*vPtr).row((*fPtr)(planeID, 0));
    Eigen::Vector3d e2 = (*vPtr).row((*fPtr)(planeID, 2)) - (*vPtr).row((*fPtr)(planeID, 0));
    Eigen::Vector3d normal = (e1.cross(e2)).normalized();
    double a = normal.x(), b = normal.y(), c = normal.z(), d = -normal.dot((*vPtr).row((*fPtr)(planeID, 0)));

    Eigen::MatrixXd Q_i = (Eigen::MatrixXd(4, 4) <<
        a * a, a * b, a * c, a * d,
        b * a, b * b, b * c, b * d,
        c * a, c * b, c * c, c * d,
        d * a, d * b, d * c, d * d).finished();

    (*facesQPtr)[planeID] = Q_i;
}

void SimplificationSolver::computeVaildEdges() {

    for (int f = 0; f < (*fPtr).rows(); f++) {

        Edge e1 = Edge((*fPtr)(f, 0), (*fPtr)(f, 1), vPtr, relationPtr, facesQPtr);
        Edge e2 = Edge((*fPtr)(f, 0), (*fPtr)(f, 2), vPtr, relationPtr, facesQPtr);
        Edge e3 = Edge((*fPtr)(f, 1), (*fPtr)(f, 2), vPtr, relationPtr, facesQPtr);

        edges.insert(e1);
        edges.insert(e2);
        edges.insert(e3);
    }
}


bool SimplificationSolver::findBoundary() {
    std::unordered_map<Edge, int, Edge::EdgeHash> edgeUsed;
    for (int f = 0; f < faces.rows(); f++)
    {
        std::vector<Edge> edges = { Edge(faces(f, 0), faces(f, 1), vPtr, relationPtr, facesQPtr),
                                    Edge(faces(f, 1), faces(f, 2), vPtr, relationPtr, facesQPtr),
                                    Edge(faces(f, 2), faces(f, 0), vPtr, relationPtr, facesQPtr) };

        for (auto e : edges) {
            auto it = edgeUsed.find(e);
            if (it == edgeUsed.end()) {
                edgeUsed[e] = 1;
            }
            else {
                it->second++;
            }
        }
    }

    for (auto it = edgeUsed.begin(); it != edgeUsed.end(); ++it) {
        if (it->second == 1) {
            return true;
        }
    }
    return false;
}

void SimplificationSolver::prepare() {
    for (int f = 0; f < (*fPtr).rows(); f++) {
        // initialQ
        updatePlaneInfo(f);

        for (int c = 0; c < 3; c++) {
            int vertexId = (*fPtr)(f, c);

            //add associate triangle to associateFacesGraph
            (*relationPtr)[vertexId].planes.push_back(f);

            //add neighbor index to neighborsGraph
            for (int i = 0; i < 3; i++) {
                int neighborId = (*fPtr)(f, i);
                if (vertexId != neighborId) {
                    //make every neighbor unique in each neighborsSet in neighborsGraph 
                    auto it = std::find((*relationPtr)[vertexId].neighbors.begin(), (*relationPtr)[vertexId].neighbors.end(), neighborId);
                    if (it == (*relationPtr)[vertexId].neighbors.end()) (*relationPtr)[vertexId].neighbors.push_back(neighborId);
                }
            }
        }
    }
};
inline void SimplificationSolver::updateFaces(int graphID, int oldID, int newID, ContractRecord& cr) {
    for (auto planeID : (*relationPtr)[graphID].planes) {
        for (int c = 0; c < 3; c++) {
            if ((*fPtr)(planeID, c) == oldID) {
                cr.addFacesIndex({ planeID, c });
                (*fPtr)(planeID, c) = newID;
            }
        }
    }
}
inline void SimplificationSolver::updateRelation(int graphID, int removeID) {
    //remove plane which is not formated by three different vertices
    std::vector<int> toRemove = std::vector<int>();
    for (auto planeID : (*relationPtr)[graphID].planes) {
        if ((*fPtr)(planeID, 0) == (*fPtr)(planeID, 1) ||
            (*fPtr)(planeID, 0) == (*fPtr)(planeID, 2) ||
            (*fPtr)(planeID, 1) == (*fPtr)(planeID, 2))
        {
            toRemove.push_back(planeID);
        }
    }
    for (auto r : toRemove) {
        auto it = std::find((*relationPtr)[graphID].planes.begin(), (*relationPtr)[graphID].planes.end(), r);
        if (it != (*relationPtr)[graphID].planes.end()) (*relationPtr)[graphID].planes.erase(it);
    }

    //remove vertex
    auto it = std::find((*relationPtr)[graphID].neighbors.begin(), (*relationPtr)[graphID].neighbors.end(), removeID);
    if (it != (*relationPtr)[graphID].neighbors.end()) (*relationPtr)[graphID].neighbors.erase(it);
}

inline void SimplificationSolver::combineRelation(int updateID, int removeID) {
    for (auto& planeID : (*relationPtr)[removeID].planes) {
        auto it = std::find((*relationPtr)[updateID].planes.begin(), (*relationPtr)[updateID].planes.end(), planeID);
        if (it == (*relationPtr)[updateID].planes.end()) (*relationPtr)[updateID].planes.push_back(std::move(planeID));
    }
    for (auto& vertexID : (*relationPtr)[removeID].neighbors) {
        auto it = std::find((*relationPtr)[updateID].neighbors.begin(), (*relationPtr)[updateID].neighbors.end(), vertexID);
        if (it == (*relationPtr)[updateID].neighbors.end()) (*relationPtr)[updateID].neighbors.push_back(std::move(vertexID));
    }
    (*relationPtr)[removeID].planes.clear();
    (*relationPtr)[removeID].neighbors.clear();
}

inline void SimplificationSolver::removeEdges(const Edge edge) {

    for (auto i : (*relationPtr)[edge.firstIndex].neighbors) {
        for (auto j : (*relationPtr)[i].neighbors) {
            Edge e = Edge(i, j, vPtr, relationPtr, facesQPtr);
            edges.erase(e);
        }
    }
    for (auto i : (*relationPtr)[edge.secondIndex].neighbors) {
        for (auto j : (*relationPtr)[i].neighbors) {
            Edge e = Edge(i, j, vPtr, relationPtr, facesQPtr);
            edges.erase(e);
        }
    }
}
inline void SimplificationSolver::insertEdges(const Edge edge) {
    for (auto i : (*relationPtr)[edge.firstIndex].neighbors) {
        for (auto j : (*relationPtr)[i].neighbors) {
            Edge e = Edge(i, j, vPtr, relationPtr, facesQPtr);
            edges.insert(e);
        }
    }
}

void SimplificationSolver::contractEdge(const Edge edge) {

    NewVertexInfo info = edge.newVertexInfo;

    Eigen::VectorXd newValue = (Eigen::VectorXd(3) <<
        info.newVertex.x(), info.newVertex.y(), info.newVertex.z()).finished();

    ContractRecord cr = ContractRecord(edge.firstIndex,
        edge.secondIndex,
        (*vPtr).row(edge.firstIndex),
        (*vPtr).row(edge.secondIndex),
        newValue);

    removeEdges(edge);


    (*vPtr).row(edge.firstIndex) = newValue;

    (*vPtr).row(edge.secondIndex) = Eigen::VectorXd::Zero(3);

    //update associate faces' vertices
    updateFaces(edge.firstIndex, edge.secondIndex, edge.firstIndex, cr);
    updateFaces(edge.secondIndex, edge.secondIndex, edge.firstIndex, cr);

    records.push_back(cr);

    for (auto n : (*relationPtr)[edge.secondIndex].neighbors) {
        auto it = std::find((*relationPtr)[n].neighbors.begin(), (*relationPtr)[n].neighbors.end(), edge.firstIndex);

        if (it == (*relationPtr)[n].neighbors.end() && n != edge.firstIndex)
            (*relationPtr)[n].neighbors.push_back(edge.firstIndex);
    }

    //remove invaild vertex and palnes for all neighbors
    for (auto n : (*relationPtr)[edge.firstIndex].neighbors) {
        updateRelation(n, edge.secondIndex);
    }
    for (auto n : (*relationPtr)[edge.secondIndex].neighbors) {
        updateRelation(n, edge.secondIndex);
    }

    //remove invaild vertex and palnes for two end points
    updateRelation(edge.firstIndex, edge.secondIndex);
    updateRelation(edge.secondIndex, edge.firstIndex);

    combineRelation(edge.firstIndex, edge.secondIndex);

    for (auto planeID : (*relationPtr)[edge.firstIndex].planes) {
        updatePlaneInfo(planeID);
    }

    insertEdges(edge);
}

void SimplificationSolver::simplify(float& progress) {

    for (int i = 0; i < 50; i++) {
        if (edges.size() <= 0) {
            break;
        }
        auto minErr = edges.begin();
        contractEdge(*minErr);
        progress = (initialEdges - edges.size()) / float(initialEdges);

    }

}
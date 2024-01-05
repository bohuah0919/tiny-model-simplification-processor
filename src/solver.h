#pragma once
#include"geometry.h"

struct ContractRecord {
    int updateIndex;
    int removeIndex;
    Eigen::VectorXd oldValue1;
    Eigen::VectorXd oldValue2;
    Eigen::VectorXd newValue;
    std::vector<std::vector<int>> updatedFacesIndexSet;
    ContractRecord(int update, int remove, Eigen::VectorXd ov1, Eigen::VectorXd ov2, Eigen::VectorXd nv)
        : updateIndex(update), removeIndex(remove), oldValue1(ov1), oldValue2(ov2), newValue(nv) {
        updatedFacesIndexSet = std::vector<std::vector<int>>();
    }

    void addFacesIndex(std::vector<int> index) {
        assert(index.size() == 2);
        updatedFacesIndexSet.push_back(index);
    }
};

class SimplificationSolver {
public:
    SimplificationSolver();
    SimplificationSolver(Eigen::MatrixXd V, Eigen::MatrixXi F);
    void simplify(float& progress);
    inline int getMaxIter() const { return records.size() - 1; };
    void setIterationState(int targetIterID, Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    inline bool hasOpenBoundary() const { return hasBoundary; }
    void getCleanedData(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

private:

    void prepare();
    void computeVaildEdges();
    bool findBoundary();
    inline void updatePlaneInfo(int planeID);
    void contractEdge(const Edge edge);
    inline void removeEdges(const Edge edge);
    inline void insertEdges(const Edge edge);
    inline void updateFaces(int graphID, int oldID, int newID, ContractRecord& cr);
    inline void updateRelation(int graphID, int removeID);
    inline void combineRelation(int updateID, int removeID);
    inline void backTrace(int recordID);
    inline void frontIterate(int recordID);

    int initialEdges;
    int startIterID;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    Eigen::MatrixXd outVertices;
    Eigen::MatrixXi outFaces;

    bool hasBoundary;
    std::vector<int> vaildVertices;
    std::set<Edge, Edge::EdgeComparator> edges;
    std::vector<ContractRecord> records;

    std::shared_ptr<Eigen::MatrixXd> vPtr;
    std::shared_ptr<Eigen::MatrixXi> fPtr;
    std::shared_ptr<std::vector<Relation>> relationPtr;
    std::shared_ptr<std::vector<Eigen::MatrixXd>> facesQPtr;

};
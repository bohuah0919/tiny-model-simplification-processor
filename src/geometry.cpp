#include"geometry.h"

Edge::Edge(int first, int second,
    std::shared_ptr<Eigen::MatrixXd> vp,
    std::shared_ptr<std::vector<Relation>> rp,
    std::shared_ptr<std::vector<Eigen::MatrixXd>> fp) {
    if (first <= second) {
        firstIndex = first;
        secondIndex = second;
    }
    else {
        firstIndex = second;
        secondIndex = first;
    }
    vPtr = vp;
    relationPtr = rp;
    facesQPtr = fp;
    updateNewVertexInfo();
}

bool Edge::operator==(const Edge& other) const {

    return firstIndex == other.firstIndex && secondIndex == other.secondIndex;
}


void Edge::updateNewVertexInfo() {
    Eigen::MatrixXd Q12 = Eigen::MatrixXd::Zero(4, 4);

    for (int facesID : (*relationPtr)[firstIndex].planes) {
        Q12 += (*facesQPtr)[facesID];
    }


    for (int facesID : (*relationPtr)[secondIndex].planes) {
        Q12 += (*facesQPtr)[facesID];
    }

    Eigen::MatrixXd A = (Eigen::MatrixXd(4, 4) <<
        Q12(0, 0), Q12(0, 1), Q12(0, 2), Q12(0, 3),
        Q12(0, 1), Q12(1, 1), Q12(1, 2), Q12(1, 3),
        Q12(0, 2), Q12(1, 2), Q12(2, 2), Q12(2, 3),
        0.0, 0.0, 0.0, 1.0).finished();

    Eigen::FullPivLU<Eigen::Matrix4d> lu(A);

    if (lu.isInvertible()) {
        newVertexInfo.newVertex = lu.inverse() * (Eigen::VectorXd(4) << 0.0, 0.0, 0.0, 1.0).finished();
    }
    else {
        Eigen::VectorXd midPoint = (vPtr->row(firstIndex) + vPtr->row(secondIndex)) * 0.5;
        newVertexInfo.newVertex = (Eigen::VectorXd(4) << midPoint.x(), midPoint.y(), midPoint.z(), 1.0).finished();
    }
    newVertexInfo.QEM = (newVertexInfo.newVertex.transpose() * Q12 * newVertexInfo.newVertex)(0, 0);

}
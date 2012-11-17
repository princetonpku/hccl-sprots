#ifndef HCCL_DEFORMATIONGRAPH_H_
#define HCCL_DEFORMATIONGRAPH_H_

#include "VectorQuaternion.h"

#include <vector>

#include "kdtree.hpp"
typedef std::pair<Vector3d, int> DG_IndexedPoint;
typedef KDTree::KDTree<3, DG_IndexedPoint, std::pointer_to_binary_function<DG_IndexedPoint, size_t, double> > DG_KDTree;

class CTriMesh;
class KinectMesh;

class DeformationGraph
{
public:
	DeformationGraph(void);
	DeformationGraph(CTriMesh* _mesh);
	~DeformationGraph(void);

// User Interface
public:
	void Clear(void);
	void SetMesh(CTriMesh* _mesh);
	void BuildGraph(double sampling_rate = 0.2, int k = 2);
	void BuildGraph(const std::vector<std::vector<double>>& dmap, int k = 2);
	// void SetDeformationConstraints(something);
	// FUNC* SetEnergyFunctional();
	// void Solve();

// Methods
public:
	void FindClosestPoint(Vector3d ref, int* idx, int n = 1, Vector3d* pt = NULL) const;

	void Render(void);

protected:
	void BuildKDTree(void);
	void DestroyKDTree(void);

// Accessors
public:
	void GetNeighbors(int i, std::vector<int>& idx) const;

protected:
	CTriMesh* mesh;

public:
	std::vector<Vector3d> nodes;
	std::vector<Vector3d> draw_nodes;
	std::vector<Vector2i> edges;
	std::vector<std::vector<int>> nodes_neighboring_nodes;
	
	std::vector<int> node_indxes;

	DG_KDTree* kdtree;
};

#endif // HCCL_DEFORMATIONGRAPH_H_
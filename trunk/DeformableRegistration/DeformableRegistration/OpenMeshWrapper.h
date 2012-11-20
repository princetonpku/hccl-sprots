#ifndef HCCL_OPENMESHWRAPPER_H_
#define HCCL_OPENMESHWRAPPER_H_

#include <vector>
#include "VectorQuaternion.h"

// OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

struct HCCLTraits : public OpenMesh::DefaultTraits
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
	typedef OpenMesh::Vec3d Color;

	VertexAttributes (
		OpenMesh::Attributes::Status |
		OpenMesh::Attributes::Normal |
		OpenMesh::Attributes::Color);
	FaceAttributes (
		OpenMesh::Attributes::Status |
		OpenMesh::Attributes::Normal |
		OpenMesh::Attributes::Color);
	EdgeAttributes(OpenMesh::Attributes::Status);

};
typedef OpenMesh::TriMesh_ArrayKernelT<HCCLTraits> HCCLMesh;


/* 3rd party libraries */
// libkdtree++
#include "kdtree.hpp"
typedef std::pair<HCCLMesh::Point, int> IndexedPoint;
typedef KDTree::KDTree<3, IndexedPoint, std::pointer_to_binary_function<IndexedPoint, size_t, double> > HCCLKDTree;

// Geodesic
#include <geodesic_algorithm_exact.h>


#define TM_SAMPLE_UNIFORM_DART 0x0001

static Vector3d cast_to_Vector3d(const HCCLMesh::Point& pt) { return Vector3d(pt[0], pt[1], pt[2]); }

class CTriMesh : public HCCLMesh
{
public:
	CTriMesh();
	~CTriMesh();

public: 
	void Clear();
//////////////////////////////////////////////////////////////////////////
// File I/O																//
//////////////////////////////////////////////////////////////////////////
public:
	bool Read(std::string strFilePath);
	bool Write(std::string strFilePath) const;


//////////////////////////////////////////////////////////////////////////
// OpenGL Display														//
//////////////////////////////////////////////////////////////////////////
public:
	enum renderMethod {RENDER_FACES, RENDER_POINTS, RENDER_WIRE};
	bool render_flag[3];
	void Render(bool isSmooth = true, bool isVertexColor = false);
	void Draw_BoundingBox(void);
	void Draw_BoundingSphere(void); // <-- TODO

protected:
	void RenderPoints(unsigned int nFlag = 0) const;
	void RenderWireframe(unsigned int nFlag = 0) const;
	void RenderFlat(unsigned int nFlag = 0) const;
	void RenderSmooth(unsigned int nFlag = 0) const; 	

//////////////////////////////////////////////////////////////////////////
// Geometric properties													//
//////////////////////////////////////////////////////////////////////////
public:
	HCCLMesh::Point cog;
	HCCLMesh::Point bounding_box_min, bounding_box_max;
	double bounding_sphere_rad;

	void UpdateProperties(void);
	void UpdateCOG(void);
	void UpdateBoundingBox(void);
	void UpdateBoundingSphere(void);

	double GetBoundingSphereRadius(void) const;

//////////////////////////////////////////////////////////////////////////
// Rigid-Body Transformations											//
//////////////////////////////////////////////////////////////////////////
public:
	void Translate(double x, double y, double z);
	void Translate(Vector3d v);
	void Translate(OpenMesh::Vec3d v);
	void Rotate(double angle, HCCLMesh::Point axis); // <-- TODO


//////////////////////////////////////////////////////////////////////////
// Sampling methods														//
//////////////////////////////////////////////////////////////////////////
public:
	void SampleRandom(int nSamples, std::vector<Vector3d>& samples) const;
	void SampleUniform(int nSamples, std::vector<Vector3d>& samples, uint nFlag = TM_SAMPLE_UNIFORM_DART) const;
	void SampleRandom(int nSamples, std::vector<Vector3d>& samples, std::vector<int>& indx) const;
	void SampleUniform(int nSamples, std::vector<Vector3d>& samples, std::vector<int>& indx, uint nFlag = TM_SAMPLE_UNIFORM_DART) const;
	//void SampleEx(int nSamples, std::vector<Vector3d>& samples) const;
	

//////////////////////////////////////////////////////////////////////////
// Find closest point via KD-Tree search								//
//////////////////////////////////////////////////////////////////////////
public:
	HCCLKDTree* kdtree;
	void BuildKDTree(void);
	void FindClosestPoint(Vector3d ref, int* idx, int n = 1, Vector3d* pt = NULL) const;
	void DestroyKDTree(void);
	
//////////////////////////////////////////////////////////////////////////
// Geodesic Part														//
//////////////////////////////////////////////////////////////////////////
public:
	geodesic::Mesh mesh;
	geodesic::GeodesicAlgorithmExact algorithm_exact;

	std::vector<geodesic::SurfacePoint> geo_path;
	std::vector<double> geodesic_dist;
	double max_geodesic_dist;

	void InitGeo();
	void Propagate(const int single_source);
	void Propagate(const std::vector<int>& from);	
	void GetGeodesicDistanceAll();
	double GetGeodesicPath(const int to);

private:	
	void GetGeodesicDistance(const int to, double& dist);
	void HSV2RGB(double h, double s, double v, double *r, double *g, double *b);
	

//////////////////////////////////////////////////////////////////////////
// Other functions														//
//////////////////////////////////////////////////////////////////////////
public:
	void Decimate(double p); // Decimate mesh while retaining p% of vertices. (0 < p < 1)	
};



#endif // HCCL_OPENMESHWRAPPER_H_
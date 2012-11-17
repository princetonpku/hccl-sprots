#ifndef VIEWER_H
#define VIEWER_H

#include <QGLViewer/qglviewer.h>

#include <vector>

#include "OpenMeshWrapper.h"
#include "DeformationGraph.h"
#include "lbfgsb.h"

#include <geodesic_algorithm_exact.h>


class DeformableRegistration;
class Viewer : public QGLViewer
{
    Q_OBJECT

public:
	Viewer(QWidget *parent = 0);
	DeformableRegistration* pParentDlg;

protected :
	virtual void init();
	virtual void draw();
	virtual void drawWithNames();

	virtual void mousePressEvent(QMouseEvent* e);
	virtual void mouseMoveEvent(QMouseEvent *e);
	virtual void mouseReleaseEvent(QMouseEvent *e);
	bool onDrag;
	Qt::MouseButton btn_pressed;
	qglviewer::Vec mouse_curr;
	qglviewer::Vec mouse_prev;

public:
	CTriMesh templ;
	CTriMesh target;
	DeformationGraph graph, target_dmap;

	bool onEmbededDeformation;
	std::vector<int> selected_vertex_idx;
	std::vector<int> selected_handle_idx;
	std::vector<Vector3d> moved_point;

	std::vector<std::vector<int>> k_nearest_idx;
	std::vector<std::vector<double>> weight_value;

	std::vector<Vector3d> result_translation;
	std::vector<std::vector<double>> result_rotation;

	int nearest_k; // for k-nearest nodes of deformation graph

	void InitOptimization();
	void RunOptimization();
	void Deform(const CTriMesh& ori, CTriMesh& mesh, DeformationGraph& dgraph);


	/////////////////
	// Hoa Li part //
	/////////////////
	int xpt, ypt, x_res, y_res;
	std::vector<std::vector<int>> indx_source, indx_target;
	std::vector<std::vector<double>> depth_map;
	std::vector<std::vector<double>> pdx_map;
	std::vector<std::vector<double>> pdy_map;
	std::vector<std::vector<int>> check_map;
	std::vector<std::vector<std::vector<double>>> coef_map;
	std::vector<std::vector<int>> table;
	void InitOptimization_HaoLi();
	void RunOptimization_HaoLi();
	void IterativeImprovment();
	void LoadDepthMapParameters(const char* filename, std::vector<std::vector<int>>& indx);
	void LoadMat(const char* filename, std::vector<std::vector<double>>& dmap);
	void LoadC(const char* filename, std::vector<std::vector<std::vector<double>>>& c);

	int n_node;
	ap::real_1d_array x;
	ap::integer_1d_array nbd;
	ap::real_1d_array lbd, ubd;
	double epsg;
	double epsf; // 10^-9
	double epsx;
	int maxits;

	bool is_hoa_initialized;

	//////////////////
	// Geodesic part//
	//////////////////
	geodesic::Mesh mesh;
	

	bool is_geo;
	unsigned source_vertex_index;
	unsigned target_vertex_index;
	Vector3d src_pt, trgt_pt;
	std::vector<Vector3d> geodesic_path;
	std::vector<Vector3d> geodesic_color;
	void InitGeo();
	void GeodesicTem();
	double GeodesicTem(geodesic::Mesh& mesh, const int src_indx, const int trgt_indx, std::vector<Vector3d>& path);
};

#endif // VIEWER_H
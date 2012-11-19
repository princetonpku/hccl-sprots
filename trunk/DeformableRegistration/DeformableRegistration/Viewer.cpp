#include "Viewer.h"

#include <QMouseEvent>
#include <limits>

#include "glPrimitives.h"
#include "DeformableRegistration.h"

#define ARMA_DONT_USE_BLAS
#include <armadillo>

using namespace std;

Viewer::Viewer(QWidget *parent) : QGLViewer(parent)
	, btn_pressed(Qt::NoButton)
	, onDrag(false)
	, onEmbededDeformation(false), is_hoa_initialized(false), is_geo(false)
{
}


void Viewer::Clear()
{
	onEmbededDeformation = false;
	is_hoa_initialized = false;
	is_geo = false;

	templ.Clear();
	target.Clear();
	graph.Clear();
	target_dmap.Clear();

	selected_vertex_idx.clear();
	selected_handle_idx.clear();
	moved_point.clear();
	k_nearest_idx.clear();
	weight_value.clear();
	result_translation.clear();
	result_rotation.clear();

}


void Viewer::init()
{
	// Restore previous viewer state.
	//restoreStateFromFile();

// 	setMouseBinding(Qt::LeftButton, SELECT);
// 	setMouseBinding(Qt::LeftButton + Qt::CTRL, SELECT);
// 	setMouseBinding(Qt::LeftButton + Qt::ALT, SELECT);
// 	setMouseBinding(Qt::LeftButton, NO_CLICK_ACTION, true);
// 	setMouseBinding(Qt::LeftButton + Qt::CTRL, NO_CLICK_ACTION, true);
// 	setMouseBinding(Qt::LeftButton + Qt::ALT, NO_CLICK_ACTION, true);
// 
// 	setMouseBinding(Qt::MidButton, CAMERA, ROTATE);
// 	//setMouseBinding(Qt::MidButton + Qt::CTRL, FRAME, ROTATE);
// 	setMouseBinding(Qt::MidButton, ALIGN_CAMERA, true);
// 	//setMouseBinding(Qt::MidButton + Qt::CTRL, ALIGN_FRAME, true);

	glClearColor(1.f, 1.f, 1.f, 1.f);
	camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);	
	setSceneRadius(100.0);

// 	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 	glEnable(GL_DEPTH_TEST);
// 	glClearDepth(1.0);
// 	glDepthFunc(GL_LEQUAL);
// 	glAlphaFunc(GL_GREATER,0.1);
// 	glEnable(GL_ALPHA_TEST);
// 	glEnable(GL_CULL_FACE);
}

void Viewer::draw()
{
	glTranslated(-templ.cog[0], -templ.cog[1], -templ.cog[2]);

	// Draw Geodesic path
	if(is_geo)
	{
		double r = templ.GetBoundingSphereRadius();

		glColor4ub(255,100,190,200);
		glPushMatrix();
		glTranslatev(templ.point(templ.vertex_handle(source_vertex_index)).data());
		DrawSphere(0.02*r);
		glPopMatrix();

		glColor4ub(100,190,255,200);
		glPushMatrix();
		glTranslatev(templ.point(templ.vertex_handle(target_vertex_index)).data());
		DrawSphere(0.02*r);
		glPopMatrix();

		glColor3ub(255,0,0);
		for (int i = 0; i<geodesic_path.size(); ++i)
		{
			glPushMatrix();
			glTranslatev(geodesic_path[i].val);
			DrawSphere(0.007*r);
			glPopMatrix();
		}
		glEnable(GL_LINE_SMOOTH);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i<geodesic_path.size(); ++i)
		{
			glVertex3dv(geodesic_path[i].val);
		}
		glEnd();
		glDisable(GL_LINE_SMOOTH);
	}



	// Draw Deformation graph
// 	if(pParentDlg->ui.actionDeformationGraph->isChecked())
// 	{
// 		glHint(GL_POINT_SMOOTH_HINT, GL_NICEST );
// 		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
// 		glEnable(GL_POINT_SMOOTH);
// 		glEnable(GL_LINE_SMOOTH);
// 				
// 		glColor3ub(0,0,0);
// 		glPointSize(10.0);
// 		glLineWidth(2.0);
// 		glEnable(GL_DEPTH_TEST);
// 		graph.Render();
// 
// 		glDisable(GL_POINT_SMOOTH);
// 		glDisable(GL_LINE_SMOOTH);
// 	}

	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Draw Deformation graph
	if(pParentDlg->ui.actionDeformationGraph->isChecked())
	{
		glColor4ub(60,120,60,200);
		for(int i = 0; i < graph.draw_nodes.size(); i++)
		{
			glPushMatrix();
			glTranslatev(graph.draw_nodes[i].val);
			if (i==100) DrawSphere(10);
			else DrawSphere(2);
			glPopMatrix();
		}

		glLineWidth(2.0);
		glBegin(GL_LINES);
		for(int i = 0; i < graph.edges.size(); i++)
		{
			glVertex3dv(graph.draw_nodes[graph.edges[i][0]].val);
			glVertex3dv(graph.draw_nodes[graph.edges[i][1]].val);
		}
		glEnd();

		// Draw corresponding points
		if (is_hoa_initialized)
		{
			glColor4ub(180,30,150,200);
			vector<int> indx;
			for(int i = 0; i < graph.draw_nodes.size(); i++)
			{
				double au, av, d, d1, d2, u = x(i*15+13), v = x(i*15+14);
				au = u-int(u);
				av = v-int(v);
				if (u+1>=x_res-1 || v+1>=y_res-1)
				{
					d = depth_map[v][u];
					indx.push_back(int(v)*x_res+int(u));
				}
				else
				{
					d1 = (1-au)*depth_map[int(v)][int(u)] + au*depth_map[int(v)][int(u)+1];
					d2 = (1-au)*depth_map[int(v)+1][int(u)] + au*depth_map[int(v)+1][int(u)+1];
					d = (1-av)*d1+av*d2;
				}

				Vector3d c_pt(x(i*15+13),x(i*15+14),depth_map[x(i*15+14)][x(i*15+13)]);
				glPushMatrix();
				glTranslatev(c_pt.val);
				DrawSphere(2);
				glPopMatrix();

				glBegin(GL_LINES);
				glVertex3dv(graph.draw_nodes[i].val);
				glVertex3dv(c_pt.val);
				glEnd();
			}
// 			cout<<"num of near edge pts"<<endl;
// 			cout<<indx.size()<<endl<<endl;
			glColor3ub(0,0,255);
			for (int i = 0; i<indx.size(); ++i)
			{
				glPushMatrix();
				glTranslatev(target_dmap.nodes[indx[i]]);
				DrawSphere(3);
				glPopMatrix();
			}
// 			glColor4ub(120,30,180,150);
// 			for (int i = 0; i<target_dmap.nodes.size(); ++i)
// 			{
// 				glPushMatrix();
// 				glTranslatev(target_dmap.nodes[i]);
// 				DrawSphere(1.7);
// 				glPopMatrix();
// 			}
		}
	}

	// Draw Template Mesh
	if(pParentDlg->ui.actionTemplateVisible->isChecked())
	{
		// 	glColor4ub(255, 190, 100, 200);
		if (pParentDlg->ui.actionSmooth->isChecked())
		{
			if (is_geo)
			{
				glEnableClientState(GL_VERTEX_ARRAY);
				glVertexPointer(3, GL_DOUBLE, 0, templ.points());

				glEnableClientState(GL_NORMAL_ARRAY);
				glNormalPointer(GL_DOUBLE, 0, templ.vertex_normals());	

				glEnableClientState(GL_COLOR_ARRAY);
				glColorPointer(3, GL_DOUBLE, 0, templ.vertex_colors());

				HCCLMesh::ConstFaceIter fIt(templ.faces_begin()), fEnd(templ.faces_end());
				HCCLMesh::ConstFaceVertexIter fvIt;

				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glBegin(GL_TRIANGLES);
				for (; fIt!=fEnd; ++fIt)
				{
					fvIt = templ.cfv_iter(fIt.handle());
					glArrayElement(fvIt.handle().idx());
					glArrayElement((++fvIt).handle().idx());
					glArrayElement((++fvIt).handle().idx());
				}
				glEnd();

				glDisableClientState(GL_VERTEX_ARRAY);
				glDisableClientState(GL_NORMAL_ARRAY);
				glDisableClientState(GL_COLOR_ARRAY);
			}
			else
			{
				glColor3ub(255, 190, 100);
				templ.Render(CTriMesh::RENDER_SMOOTH);
			}			
		}

		if (pParentDlg->ui.actionPoint->isChecked())
		{
			glColor3ub(0, 0, 0);
			templ.Render(CTriMesh::RENDER_POINTS);
		}			

		if (pParentDlg->ui.actionWireframe->isChecked())
		{
			glColor3ub(0, 0, 0);
			templ.Render(CTriMesh::RENDER_WIRE);
		}
			
			

// 		glColor3ub(0, 0, 0);
// 		templ.Render(CTriMesh::RENDER_WIRE);
	}

	// Draw Target Mesh
	if(pParentDlg->ui.actionTargetVisible->isChecked())
	{
		if(onEmbededDeformation)
			glColor3ub(255, 190, 100);
		else
			glColor3ub(100, 190, 255);
		target.Render();
	}

	// Draw selected points
	{		
		double r = templ.GetBoundingSphereRadius();
		for(size_t i = 0; i < selected_vertex_idx.size(); i++)
		{
			glPushMatrix();
			glTranslatev(moved_point[i]);
			if(std::find(selected_handle_idx.begin(), selected_handle_idx.end(), i) != selected_handle_idx.end())
				glColor4ub(255,100,190,200);
			else
				glColor4ub(100,190,255,200);
			DrawSphere(0.03*r);
			glPopMatrix();
		}
	}
}

void Viewer::drawWithNames()
{
	glDisable(GL_CULL_FACE);
	double r = templ.GetBoundingSphereRadius();
	for(size_t i = 0; i < selected_vertex_idx.size(); i++)
	{
		glPushName(-2-i);
		glPushMatrix();
		//glTranslatev(cast_to_Vector3d(templ.point(templ.vertex_handle(selected_vertex_idx[i]))));
		glTranslatev(moved_point[i]);
		DrawSphere(0.03*r);
		glPopMatrix();
		glPopName();
	}
	HCCLMesh::ConstFaceIter fIt(target.faces_begin()), fEnd(target.faces_end());
	HCCLMesh::ConstFaceVertexIter fvIt;
	for(; fIt != fEnd; ++fIt)
	{
		glPushName(fIt.handle().idx());
		glBegin(GL_TRIANGLES);
		fvIt = target.cfv_iter(fIt);
		glNormal3dv(target.normal(fIt).data());
		glVertex3dv(target.point(fvIt.handle()).data());
		glVertex3dv(target.point((--fvIt).handle()).data());
		glVertex3dv(target.point((--fvIt).handle()).data());
		glEnd();
		glPopName();
	}
	glEnable(GL_CULL_FACE);

}

void Viewer::mousePressEvent(QMouseEvent* e)
{
	if ((e->button() == Qt::LeftButton) && onEmbededDeformation)
	{
		QGLViewer::mousePressEvent(e);
		int sel_facet = selectedName();
		if(sel_facet < -1 && e->modifiers() == Qt::NoModifier)
		{
			int n = -sel_facet-2;

			if(selected_handle_idx.size() == 0)
			{
				std::vector<int>::iterator it;
				if( (it = std::find(selected_handle_idx.begin(), selected_handle_idx.end(), n)) == selected_handle_idx.end())
					selected_handle_idx.push_back(n);
			}
			else
			{
				std::vector<int>::iterator it;
				if( (it = std::find(selected_handle_idx.begin(), selected_handle_idx.end(), n)) == selected_handle_idx.end())
				{
					selected_handle_idx.clear();
					selected_handle_idx.push_back(n);
				}
			}
		}
		mouse_prev = qglviewer::Vec(e->x(), e->y(), 0);
		btn_pressed = Qt::LeftButton;
	}
	else
	{
		QGLViewer::mousePressEvent(e);
	}
}

void Viewer::mouseMoveEvent(QMouseEvent *e)
{
	if(btn_pressed == Qt::LeftButton && onEmbededDeformation)
	{
		if(selected_handle_idx.size())
		{
			mouse_curr = qglviewer::Vec(e->x(), e->y(), 0);
			qglviewer::Vec mouse_prev_unproj = camera()->unprojectedCoordinatesOf(mouse_prev);
			qglviewer::Vec mouse_curr_unproj = camera()->unprojectedCoordinatesOf(mouse_curr);
			qglviewer::Vec displacement = mouse_curr_unproj - mouse_prev_unproj;

			for(int i = 0; i < moved_point.size(); i++)
			{
				if(std::find(selected_handle_idx.begin(), selected_handle_idx.end(), i) != selected_handle_idx.end())
				{
					moved_point[i] += Vector3d(displacement.x, displacement.y, displacement.z);
				}
			}
			RunOptimization();
			onDrag = true;
			mouse_prev = mouse_curr;
			updateGL();
		}
	}
	QGLViewer::mouseMoveEvent(e);
}

void Viewer::mouseReleaseEvent(QMouseEvent *e)
{
	if ((e->button() == Qt::LeftButton) && !onDrag && onEmbededDeformation)
	{
		QGLViewer::mousePressEvent(e);
		int sel_facet = selectedName();
		if(sel_facet == -1)
		{
			if(e->modifiers() == Qt::NoModifier)
			{
				selected_vertex_idx.clear();
				selected_handle_idx.clear();
				moved_point.clear();
			}
		}
		else if(sel_facet >= 0 && sel_facet < templ.n_faces() && e->modifiers() != Qt::ALT)
		{
			if(e->modifiers() != Qt::CTRL)
			{
				selected_vertex_idx.clear();
				selected_handle_idx.clear();
				moved_point.clear();
			}
			selected_handle_idx.clear();

			HCCLMesh::ConstFaceVertexIter fvit = templ.cfv_iter(templ.face_handle(sel_facet));

			int min_idx = 0;
			double min_val = std::numeric_limits<double>::infinity();
			HCCLMesh::Point pt;
			qglviewer::Vec pt_;
			qglviewer::Vec s;

			pt = templ.point(fvit);
			pt_[0] = pt[0];		pt_[1] = pt[1];		pt_[2] = pt[2];
			s = camera()->projectedCoordinatesOf(pt_);
			min_val = (s[0] - e->x())*(s[0] - e->x()) + (s[1] - e->y())*(s[1] - e->y());
			min_idx = fvit.handle().idx();

			pt = templ.point(++fvit);
			pt_[0] = pt[0];		pt_[1] = pt[1];		pt_[2] = pt[2];
			s = camera()->projectedCoordinatesOf(pt_);
			if(min_val > (s[0] - e->x())*(s[0] - e->x()) + (s[1] - e->y())*(s[1] - e->y()) )
			{
				min_val = (s[0] - e->x())*(s[0] - e->x()) + (s[1] - e->y())*(s[1] - e->y());
				min_idx = fvit.handle().idx();
			}

			pt = templ.point(++fvit);
			pt_[0] = pt[0];		pt_[1] = pt[1];		pt_[2] = pt[2];
			s = camera()->projectedCoordinatesOf(pt_);
			if(min_val > (s[0] - e->x())*(s[0] - e->x()) + (s[1] - e->y())*(s[1] - e->y()) )
			{
				min_val = (s[0] - e->x())*(s[0] - e->x()) + (s[1] - e->y())*(s[1] - e->y());
				min_idx = fvit.handle().idx();
			}
			selected_vertex_idx.push_back(min_idx);
			moved_point.push_back(cast_to_Vector3d(templ.point(templ.vertex_handle(min_idx))));
		}
		else if(sel_facet < -1)
		{
			int n = -sel_facet-2;

			if(e->modifiers() == Qt::NoModifier)
			{
				selected_handle_idx.clear();
			}
			else if(e->modifiers() == Qt::ALT)
			{
				selected_vertex_idx.erase(selected_vertex_idx.begin() + n);
				std::vector<int>::iterator it;
				if( (it = std::find(selected_handle_idx.begin(), selected_handle_idx.end(), n)) != selected_handle_idx.end())
					selected_handle_idx.erase(it);
			}
			else
			{
				std::vector<int>::iterator it;
				if( (it = std::find(selected_handle_idx.begin(), selected_handle_idx.end(), n)) != selected_handle_idx.end())
					selected_handle_idx.erase(it);
				else
					selected_handle_idx.push_back(n);
			}
		}
		moved_point.resize(selected_vertex_idx.size());
		updateGL();
	}

// 	if(selectedName() < -1 && onDrag)
// 	{
// 		if(e->modifiers() == Qt::NoModifier)
// 		{
// 			selected_handle_idx.clear();
// 		}
// 	}
	onDrag = false;
	btn_pressed = Qt::NoButton;
	QGLViewer::mouseReleaseEvent(e);
}

void Viewer::InitOptimization()
{
	// building graph
	graph.SetMesh(&templ);

	graph.BuildGraph(min(300.0/templ.n_vertices(), 1.0), 2);

	// the number of nodes which influence each vertex
	nearest_k = 4;
	weight_value.resize(templ.n_vertices(), vector<double>(nearest_k+1));
	k_nearest_idx.resize(templ.n_vertices(), vector<int>(nearest_k+1));

	// get weight values of each vertex
	int start_tic = clock();
//  #pragma omp parallel for
	for (int i = 0; i<weight_value.size(); ++i)
	{		
		Vector3d vertex(templ.point(templ.vertex_handle(i))[0], templ.point(templ.vertex_handle(i))[1], templ.point(templ.vertex_handle(i))[2]);
		graph.FindClosestPoint(vertex , k_nearest_idx[i].data(), nearest_k+1);
		
		for (int j = 0; j<nearest_k+1; ++j)
			weight_value[i][j] = Norm(vertex - graph.nodes[k_nearest_idx[i][j]]);

		double sum = 0;
		for (int j = 0; j<nearest_k; ++j)
		{
			double temp = 1-weight_value[i][j]/weight_value[i][nearest_k];
			weight_value[i][j] = temp*temp;
			sum += weight_value[i][j];
		}
		for (int j = 0; j<nearest_k; ++j)
			weight_value[i][j] /= sum;
	}
	graph.draw_nodes = graph.nodes;

	printf("set k-nearest node and weight values : %f sec\n", (clock()-start_tic)/double(CLOCKS_PER_SEC));	
}

void Viewer::RunOptimization()
{
	int start_tic = clock();
	int n_node = graph.nodes.size();

	ap::real_1d_array x;
	ap::integer_1d_array nbd;
	ap::real_1d_array lbd, ubd;	
	x.setbounds(1, n_node*12);
	nbd.setbounds(1, n_node*12);
	lbd.setbounds(1, n_node*12);
	ubd.setbounds(1, n_node*12);

	for(int i = 1; i <= n_node*12; i++)
	{
		x(i) = (i%12 == 1 || i%12 == 5 || i%12 == 9) ? 1 : 0;
		nbd(i) = 0;
		lbd(i) = 0.0;
		ubd(i) = 0.0;
	}

	int info = 0;

// 	lbfgsbminimize(n_node*12, 5, x, graph, templ, selected_vertex_idx, moved_point, k_nearest_idx, weight_value, 0.00001, 0.00001, 0.00001, 200, nbd, lbd, ubd, info);

	// update solution
	result_translation.resize(n_node);
	result_rotation.resize(n_node, vector<double>(9));

// #pragma parallel for
	for(int i = 0; i < n_node; ++i)
	{
		result_translation[i] = Vector3d(x(12*i+10), x(12*i+11), x(12*i+12));
		for (int j = 0; j < 9; ++j)
			result_rotation[i][j] = x(12*i+j+1);
	}
 	Deform(templ, target, graph);

// 	printf("time for one iteration : %f sec\n", (clock()-start_tic)/double(CLOCKS_PER_SEC));
}

void Viewer::Deform( const CTriMesh& ori, CTriMesh& mesh, DeformationGraph& dgraph )
{
	HCCLMesh::ConstVertexIter cvit = ori.vertices_begin();
	HCCLMesh::VertexIter vit = mesh.vertices_begin();

	// mesh update
	for (; cvit!=ori.vertices_end(); ++cvit, ++vit)
	{
		int idx_vrtx = cvit.handle().idx();

		arma::vec  v_ = arma::vec().zeros(3);		
		arma::vec  v(3);
		HCCLMesh::Point tem = ori.point(cvit);
		v[0] = tem[0];
		v[1] = tem[1];
		v[2] = tem[2];

		for (int j = 0; j<nearest_k; ++j)
		{
			int idx_node = k_nearest_idx[idx_vrtx][j];

			double w = weight_value[idx_vrtx][j];
			arma::vec g(dgraph.nodes[idx_node].val, 3);
			g[0] = dgraph.nodes[idx_node][0];
			g[1] = dgraph.nodes[idx_node][1];
			g[2] = dgraph.nodes[idx_node][2];

			arma::vec t(result_translation[idx_node].val, 3);
			t[0] = result_translation[idx_node][0];
			t[1] = result_translation[idx_node][1];
			t[2] = result_translation[idx_node][2];

			arma::mat R(result_rotation[idx_node].data(), 3, 3);

			arma::vec d = v-g;
			arma::vec rd;

			rd = R*d;		
			v_ += w*(rd+g+t);
		}

		mesh.set_point(*vit, HCCLMesh::Point(v_[0], v_[1], v_[2]));
	}

	mesh.update_normals();

	// deformation graph update
	std::transform(graph.nodes.begin(), graph.nodes.end(), result_translation.begin(), dgraph.draw_nodes.begin(), [](const Vector3d& a, const Vector3d& b){return a+b;});
}

/////////////////
// Hoa Li part //
/////////////////
// #include "lbfgsb_haoli.h"
void Viewer::InitOptimization_HaoLi()
{
	// depth map data load
	ifstream fin("target_dmap.txt");
	fin >> xpt >> ypt >> x_res >> y_res;
	depth_map.resize(y_res);
	for (int i = 0; i<y_res; ++i)
	{
		depth_map[i].resize(x_res);
		for (int j = 0; j<x_res; ++j)
		{
			fin >> depth_map[i][j];
		}		
	}
	fin.close();
	target_dmap.BuildGraph(depth_map);

	// partial derivative
	LoadMat("pdx.txt", pdx_map);
	LoadMat("pdy.txt", pdy_map);

	// load depth map parameters
	LoadDepthMapParameters("indx1.txt", indx_source);
	LoadDepthMapParameters("indx2.txt", indx_target);
	LoadC("coef_map.txt", coef_map);

	graph.Clear();
	graph.SetMesh(&templ);
	graph.BuildGraph(min(300.0/templ.n_vertices(), 1.0), 2);
	n_node = graph.nodes.size();
	cout<< "node size = "<<n_node << endl;

	// the number of nodes which influence each vertex
	nearest_k = 4;
	weight_value.resize(templ.n_vertices(), vector<double>(nearest_k+1));
	k_nearest_idx.resize(templ.n_vertices(), vector<int>(nearest_k+1));

	// get weight values of each vertex
	for (int i = 0; i<weight_value.size(); ++i)
	{
		Vector3d vertex(templ.point(templ.vertex_handle(i))[0], templ.point(templ.vertex_handle(i))[1], templ.point(templ.vertex_handle(i))[2]);
		graph.FindClosestPoint(vertex , k_nearest_idx[i].data(), nearest_k+1);

		for (int j = 0; j<nearest_k+1; ++j)
			weight_value[i][j] = Norm(vertex - graph.nodes[k_nearest_idx[i][j]]);

		double sum = 0;
		for (int j = 0; j<nearest_k; ++j)
		{
			double temp = 1-weight_value[i][j]/weight_value[i][nearest_k];
			weight_value[i][j] = temp;
			sum += weight_value[i][j];
		}

		for (int j = 0; j<nearest_k; ++j)
			weight_value[i][j] /= sum;
	}
	graph.draw_nodes = graph.nodes;
	

	// Init LBFGS

	epsg = 0.0;
	epsf = 0.000000001; // 10^-9
	epsx = 0.0;
	maxits = 3000;	

	x.setbounds(1, n_node*15);
	nbd.setbounds(1, n_node*15);
	lbd.setbounds(1, n_node*15);
	ubd.setbounds(1, n_node*15);

	for(int i = 1; i <= n_node*15; ++i)
	{
		x(i) = (i%15 == 1 || i%15 == 5 || i%15 == 9 || i%15 == 0) ? 1 : 0;

		if (i%15 == 13)
		{
			nbd(i) = 2;
			lbd(i) = 0;
			ubd(i) = x_res-2;
		}
		else if (i%15 == 14)
		{
			nbd(i) = 2;
			lbd(i) = 0;
			ubd(i) = y_res-2;
		}
		else
		{
			nbd(i) = 0;
			lbd(i) = 0.0;
			ubd(i) = 0.0;
		}

	}

	// initialize via a closest point computation
	for(int i = 0; i < n_node; i++)
	{
		int indx_tem;
		target_dmap.FindClosestPoint(graph.nodes[i], &indx_tem);
// 		x(i*15+13) = indx_target[indx_tem][0] -xpt;
// 		x(i*15+14) = indx_target[indx_tem][1] -ypt;
		x(i*15+13) = indx_tem%x_res;
		x(i*15+14) = indx_tem/x_res;
	}

	is_hoa_initialized = true;
}
void Viewer::RunOptimization_HaoLi()
{
	std::vector<double> alph(10); // condition
	bool not_converged = true;	
	cout << "start!!" << endl<<endl<<endl;
	while(not_converged)
	{
// 		alph[0] = 1000;			// e_rigid
// 		alph[1] = 100;			// e_smooth
// 		alph[2] = 0.1;			// e_fit --> fixed!
// 		alph[3] = 100;			// e_con
// 
// 		alph[4] = 1;			// e_rigid is halved
// 		alph[5] = 0.1;			// e_smooth is halved
// 		alph[6] = 1;			// e_con is halved
		alph[0] = 0;			// e_rigid
		alph[1] = 0;			// e_smooth
		alph[2] = 1;			// e_fit --> fixed!
		alph[3] = 0;			// e_con

		alph[4] = 1;			// e_rigid is halved
		alph[5] = 0.1;			// e_smooth is halved
		alph[6] = 1;			// e_con is halved


		alph[7] = 0.0000000000000000001;		// coefficient is halved condition! 10^-5
		alph[8] = 0.000000000000000000001;		// iterative improvement condition! 10^-6
		alph[9] = 0.000000000000000001;	// converge condition! 10^-8


		int info = 0;
		lbfgsbminimize(n_node*15, 5, x, graph, templ, alph, depth_map, pdx_map, pdy_map, coef_map, this, epsg, epsf, epsx, maxits, nbd, lbd, ubd, info);
		cout<<info<<endl;

		if (info==-2)
		{
			cout<< "unknown internal error." << endl;
			not_converged = false;
		}
		else if (info==-1)
		{
			cout<< "wrong parameters were specified." << endl;
			not_converged = false;
		}
		else if (info==0)
		{
			cout<< "interrupted by user." << endl;
			not_converged = false;
		}
		else if (info==1)
		{
			cout<< "relative function decreasing is less or equal to EpsF." << endl;
			not_converged = false;
		}
		else if (info==2)
		{
			cout<< "step is less or equal to EpsX." << endl;
			not_converged = false;
		}
		else if (info==4)
		{
			cout<< "gradient norm is less or equal to EpsG." << endl;
			not_converged = false;
		}
		else if (info==5)
		{
			cout<< "number of iterations exceeds MaxIts." << endl;
			not_converged = false;
		}
		else if (info==6)
		{
			cout<< "The entire process converged" << endl;
			not_converged = false;
		}
		else if (info==7)
		{
			cout<< "iterative improvement" << endl;
			cout<< "!!!!!!" << endl;
			cout<< "!!!!!!" << endl;
			cout<< "!!!!!!" << endl;

			double hole_value = 1000;

			int total = x_res*y_res;
			for (int i = 0; i<n_node; ++i)
			{
				// x_hat = x+b
				Vector3d x_hat = graph.nodes[i] + Vector3d(x(15*i+10), x(15*i+11), x(15*i+12));

				// cloest point
				int indx;
				Vector3d correspond_pt;
				target_dmap.FindClosestPoint(Vector3d(x_hat), &indx, 1, &correspond_pt);

				int xx = indx%x_res;
				int yy = indx/x_res;

				x(15*i+13) = xx;
				x(15*i+14) = yy;

// 				if (xx>x_res-1 || yy>y_res-1)
// 				{
// 					cout<<"oh no"<<endl;
// 					cout<<"oh no"<<endl;
// 					cout<<"oh no"<<endl;
// 				}
// 
// 				cout<<i<<endl;

// 				cout<<"condition 1 : hole value"<<endl;
				if (correspond_pt[2] > hole_value*0.9)
				{
					x(15*i+15) = 0;
					continue;
				}

// 				cout<<"condition 2 : distance > 2cm"<<endl;
				if (Norm(x_hat-correspond_pt) > 20.0)
				{
					x(15*i+15) = 0;
					continue;
				}

// 				// condition 3 : normal vector test
// 				pt = templ.normal(HCCLMesh::VertexHandle(graph.node_indxes[i]));				
// 				Vector3d n(x(15*i+1)*pt[0]+x(15*i+4)*pt[1]+x(15*i+7)*pt[2],
// 							x(15*i+2)*pt[0]+x(15*i+5)*pt[1]+x(15*i+8)*pt[2],
// 							x(15*i+3)*pt[0]+x(15*i+6)*pt[1]+x(15*i+9)*pt[2]);
// 				pt = target.normal(HCCLMesh::VertexHandle(indx));
// 				Vector3d correspond_normal(pt[0], pt[1], pt[2]);
// 
// 				if (n.Dot(correspond_normal) < 0.6)
// 				{
// 					x(15*i+15) = 0;
// 					continue;
// 				}

				// else : good correspondence
				x(15*i+15) = 1;
			}

		}
	}

	cout << endl << endl <<"update solution" << endl << endl;
	// update solution
	ofstream fout("result.txt");
	for(int i = 0; i < n_node; ++i)
	{
		for (int j = 1; j<=14; ++j)
		{
			fout << x(i*15+j) << ", ";
		}
		fout << x(i*15+15) <<endl;
	}
	fout.close();

	// update graph nodes
	for (int i = 0; i<n_node; ++i)
	{		
		graph.draw_nodes[i][0] += x(15*i+10);
		graph.draw_nodes[i][1] += x(15*i+11); 
		graph.draw_nodes[i][2] += x(15*i+12);		
	}
	updateGL();
}

void Viewer::LoadDepthMapParameters( const char* filename, std::vector<std::vector<int>>& indx )
{
	ifstream fin(filename);
	while(!fin.eof())
	{
		std::vector<int> tem(2);
		fin >> tem[0] >> tem[1];
		indx.push_back(tem);
	}
	fin.close();
	indx.pop_back();	
}

void Viewer::LoadMat( const char* filename, std::vector<std::vector<double>>& mat )
{
	ifstream fin(filename);
	mat.resize(y_res);
	for (int i = 0; i<y_res; ++i)
	{
		mat[i].resize(x_res);
		for (int j = 0; j<x_res; ++j)
		{
			fin >> mat[i][j];
		}		
	}
	fin.close();
}

void Viewer::LoadC( const char* filename, std::vector<std::vector<std::vector<double>>>& c )
{
	ifstream fin(filename);
	c.resize(y_res);
	for (int i = 0; i<y_res; ++i)
	{
		c[i].resize(x_res);
		for (int j = 0; j<x_res; ++j)
		{
			c[i][j].resize(6);
			for (int k = 0; k<6; ++k)
			{
				fin >> c[i][j][k];
			}
		}
	}
	fin.close();

}

void Viewer::InitGeo()
{
	cout<< "init geodesic mesh...";
	// load points and faces data
	

	//std::vector<double> points;
	std::vector<double> points(&templ.points()[0][0], &templ.points()[templ.n_vertices()][0]);

	std::vector<unsigned> faces;
	faces.reserve(templ.n_faces()*3);

	HCCLMesh::ConstFaceIter fIt(templ.faces_begin()), fEnd(templ.faces_end());
	HCCLMesh::ConstFaceVertexIter fvIt;
	for (; fIt!=fEnd; ++fIt)
	{
		fvIt = templ.cfv_iter(fIt.handle());
		faces.push_back(fvIt.handle().idx());
		faces.push_back((++fvIt).handle().idx());
		faces.push_back((++fvIt).handle().idx());
	}

	// creat mesh data
	mesh.initialize_mesh_data(points, faces);		//create internal mesh data structure including edges

	cout<<"finished!"<<endl;
}

void Viewer::GeodesicTem()
{
	is_geo = true;
	source_vertex_index = 0;
	target_vertex_index = 5;
	double g_length = GeodesicTem(mesh, source_vertex_index, target_vertex_index, geodesic_path);
	updateGL();
}

double Viewer::GeodesicTem( geodesic::Mesh& mesh, const int src_indx, const int trgt_indx, std::vector<Vector3d>& path )
{
	path.clear();

	// creat exact algorithm
	geodesic::GeodesicAlgorithmExact algorithm(&mesh);	//create exact algorithm for the mesh

	// creat source
	geodesic::SurfacePoint source(&mesh.vertices()[src_indx]);
	std::vector<geodesic::SurfacePoint> all_sources(1,source);	//in general, there could be multiple sources, but now we have only one

	//create target
	geodesic::SurfacePoint target(&mesh.vertices()[trgt_indx]);
		
	//find a single source-target path	
	std::vector<geodesic::SurfacePoint> path_tem;	//geodesic path is a sequence of SurfacePoints
	algorithm.geodesic(source, target, path_tem);
// 	algorithm.print_statistics();

	path.resize(path_tem.size());
	for(unsigned i = 0; i<path.size(); ++i)
		path[i] = Vector3d(path_tem[i].xyz());


	//cover the whole mesh
	vector<double> geo_distance(mesh.vertices().size());
	algorithm.propagate(all_sources);
// 	algorithm.print_statistics();

	for(unsigned i=0; i<mesh.vertices().size(); ++i)
	{
		geodesic::SurfacePoint p(&mesh.vertices()[i]);
		unsigned best_source = algorithm.best_source(p, geo_distance[i]);		//for a given surface point, find closets source and distance to this source
	}
	
	double mx_dist = geo_distance[std::max_element(geo_distance.begin(), geo_distance.end())-geo_distance.begin()];
	mx_dist = 5;

	auto HSVtoRGB = [](double h, double s, double v, double *r, double *g, double *b)
	{
		if( s == 0 ) {
			*r = *g = *b = v;
			return;
		}
		h /= 60;
		int i = (int)h;
		double f = h - i;
		double p = v*(1-s);
		double q = v*(1-s*f);
		double t = v*(1-s*(1-f));
		switch(i) {
		case 0: *r = v; *g = t; *b = p; break;
		case 1: *r = q; *g = v; *b = p; break;
		case 2: *r = p; *g = v; *b = t; break;
		case 3: *r = p; *g = q; *b = v; break;
		case 4: *r = t; *g = p; *b = v; break;
		default:*r = v; *g = p; *b = q; break;
		}
	};

	HCCLMesh::VertexIter vit(templ.vertices_begin()), vit_end(templ.vertices_end());
	for (; vit!=vit_end; ++vit)
	{
		double h = geo_distance[vit.handle().idx()]/mx_dist*360;
		double r,g,b;
		HSVtoRGB(h,1,1, &r, &g, &b);
		templ.set_color(vit,HCCLMesh::Color(r, g, b));		
	}

// 	print_info_about_path(path_tem);
	return length(path_tem);
}

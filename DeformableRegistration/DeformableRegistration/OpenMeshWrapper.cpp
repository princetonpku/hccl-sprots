#include "OpenMeshWrapper.h"

#ifdef QT_OPENGL_LIB
#include <qgl.h>
#else
#include <gl/GL.h>
#include <gl/GLU.h>
#endif

#include <algorithm>
#include <numeric>
#include <limits>

//#include <armadillo>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>


#ifndef min
#define min(a, b) (((a) < (b))? (a) : (b))
#endif
#ifndef max
#define max(a, b) (((a) > (b))? (a) : (b))
#endif


CTriMesh::CTriMesh(void)
	: kdtree(NULL)
{
	// 	cog[0] = cog[1] = cog[2] = 0.f;
	// 	bounding_box_min[0] = bounding_box_min[1] = bounding_box_min[2] = 0.f;
	// 	bounding_box_max[0] = bounding_box_max[1] = bounding_box_max[2] = 0.f;
	// 	bounding_sphere_rad = 0.f;
}

CTriMesh::~CTriMesh(void)
{
	DestroyKDTree();
}

void CTriMesh::Clear()
{
	this->clear();
// 	cog[0] = cog[1] = cog[2] = 0.f;
// 	bounding_box_min[0] = bounding_box_min[1] = bounding_box_min[2] = 0.f;
// 	bounding_box_max[0] = bounding_box_max[1] = bounding_box_max[2] = 0.f;
// 	bounding_sphere_rad = 0.f;
}

bool CTriMesh::Read(std::string strFilePath)
{
	OpenMesh::IO::Options opt;
	if(!has_vertex_normals())
	{
		std::cerr << "File Open Error: Standard vertex property 'Vertex Normals' not available!\n";
		return false;
	}
	if(!has_vertex_colors())
	{
		std::cerr << "File Open Error: Standard vertex property 'Vertex Colors' not available!\n";
		return false;
	}
	if(!has_face_normals())
	{
		std::cerr << "File Open Error: Standard vertex property 'Face Normals' not available!\n";
		return false;
	}
	if(!has_face_colors())
	{
		std::cerr << "File Open Error: Standard vertex property 'Face Colors' not available!\n";
		return false;
	}

	if( !OpenMesh::IO::read_mesh(*this, strFilePath, opt) )
	{
		std::cerr << "File Open Error: Error loading mesh from file " << strFilePath << std::endl;
		return false;
	}
	if( !opt.check( OpenMesh::IO::Options::FaceNormal) )
		update_face_normals();
	if( !opt.check( OpenMesh::IO::Options::VertexNormal) )
		update_vertex_normals();

	return true;
}

bool CTriMesh::Write(std::string strFilePath) const
{
	if ( !OpenMesh::IO::write_mesh(*this, strFilePath) )
	{
		std::cerr << "Cannot write mesh to file" << strFilePath << std::endl;
		return false;
	}
	return true;
}

void CTriMesh::Render( renderMethod nMethod /*= RENDER_SMOOTH*/, unsigned int nFlag /*= 0*/ ) const
{
	if (nMethod == RENDER_SMOOTH) RenderSmooth(nFlag);
	else if (nMethod == RENDER_FLAT) RenderFlat(nFlag);
	else if (nMethod == RENDER_POINTS) RenderPoints(nFlag);
	else if (nMethod == RENDER_WIRE) RenderWireframe(nFlag);
}

void CTriMesh::RenderPoints(GLuint nFlag/* = 0*/) const
{
	if(n_vertices() <= 0)
		return;

	HCCLMesh::ConstVertexIter vit(vertices_begin()), vEnd(vertices_end());

	glBegin(GL_POINTS);
	for(; vit != vEnd; ++vit)
		glVertex3dv(&point(vit)[0]);
	glEnd();
}

void CTriMesh::RenderWireframe(GLuint nFlag/* = 0*/) const
{
	if(n_vertices() <= 0)
		return;

	HCCLMesh::ConstFaceIter fIt(faces_begin()), fEnd(faces_end());
	HCCLMesh::ConstFaceVertexIter fvIt;

	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
				
	glPolygonMode(GL_FRONT_AND_BACK,  GL_LINE);
	glBegin(GL_TRIANGLES);
	for (; fIt!=fEnd; ++fIt)
	{
		fvIt = cfv_iter(fIt.handle()); 
		glVertex3dv( &point(fvIt)[0] );
		++fvIt;
		glVertex3dv( &point(fvIt)[0] );
		++fvIt;
		glVertex3dv( &point(fvIt)[0] );
	}
	glEnd();

	glDisable(GL_POINT_SMOOTH);
	glDisable(GL_LINE_SMOOTH);

}

void CTriMesh::RenderFlat(GLuint nFlag/* = 0*/) const
{
// 	HCCLMesh::FaceIter f_it, f_end(faces_end());
// 	HCCLMesh::FaceVertexIter fv_it, fv_end;
// 	HCCLMesh::Point temp;
// 	HCCLMesh::Normal normal;
// 
// 	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
// 	HCCLMesh::Color clr;
// 	for(f_it = faces_begin(); f_it != f_end; ++f_it)
// 	{		
// 		normal = this->normal(f_it);
// 		glBegin(GL_TRIANGLES);
// 		glNormal3f(normal[0], normal[1], normal[2]);
// 		for(fv_it = fv_iter(f_it); fv_it; ++fv_it)
// 		{
// 			temp = point(fv_it);
// 			clr = color(fv_it);
// 			glColor3ub( clr[0], clr[1], clr[2] );
// 			glVertex3f( temp[0], temp[1], temp[2] );
// 		}
// 		glEnd();
// 	}
}

void CTriMesh::RenderSmooth(GLuint nFlag/* = 0*/) const
{
	if(n_vertices() <= 0)
		return;
		
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_DOUBLE, 0, points());
	
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_DOUBLE, 0, vertex_normals());	

	HCCLMesh::ConstFaceIter fIt(faces_begin()), fEnd(faces_end());
	HCCLMesh::ConstFaceVertexIter fvIt;

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLES);
	for (; fIt!=fEnd; ++fIt)
	{
		fvIt = cfv_iter(fIt.handle());
		glArrayElement(fvIt.handle().idx());
		glArrayElement((++fvIt).handle().idx());
		glArrayElement((++fvIt).handle().idx());
	}
	glEnd();

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void CTriMesh::Draw_BoundingBox( void )
{
	float x = bounding_box_min[0];
	float y = bounding_box_min[1];
	float z = bounding_box_min[2];
	float w = bounding_box_max[0] - bounding_box_min[0];
	float h = bounding_box_max[1] - bounding_box_min[1];
	float l = bounding_box_max[2] - bounding_box_min[2];

	glBegin(GL_LINE_LOOP);
	glVertex3f(x, y, z);
	glVertex3f(x+w, y, z);
	glVertex3f(x+w, y+h, z);
	glVertex3f(x, y+h, z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(x, y, z+l);
	glVertex3f(x+w, y, z+l);
	glVertex3f(x+w, y+h, z+l);
	glVertex3f(x, y+h, z+l);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(x, y, z);
	glVertex3f(x, y, z+l);
	glVertex3f(x+w, y, z);
	glVertex3f(x+w, y, z+l);
	glVertex3f(x+w, y+h, z);
	glVertex3f(x+w, y+h, z+l);
	glVertex3f(x, y+h, z);
	glVertex3f(x, y+h, z+l);
	glEnd();
}

void CTriMesh::Draw_BoundingSphere( void )
{

}

void CTriMesh::UpdateProperties( void )
{
	UpdateCOG();
	UpdateBoundingBox();
	UpdateBoundingSphere();	
}

void CTriMesh::UpdateCOG( void )
{
	HCCLMesh::VertexIter v_it, v_end(vertices_end());
	cog = Point(0,0,0);

	if(n_vertices() <= 0)
	{
		std::cerr << "[calcCOG] No vertices are found.\n";
	}

	for(v_it = vertices_begin(); v_it != v_end; ++v_it)
	{
		cog += point(v_it);
	}

	cog /= n_vertices();
}

void CTriMesh::UpdateBoundingBox( void )
{
	HCCLMesh::VertexIter v_it(vertices_begin()), v_end(vertices_end());
	HCCLMesh::Point temp;

	temp = point(v_it);
	bounding_box_min[0] = bounding_box_max[0] = temp[0];
	bounding_box_min[1] = bounding_box_max[1] = temp[1];
	bounding_box_min[2] = bounding_box_max[2] = temp[2];

	for(++v_it; v_it != v_end; ++v_it)
	{
		temp = point(v_it);
		bounding_box_min[0] = min(temp[0], bounding_box_min[0]);
		bounding_box_min[1] = min(temp[1], bounding_box_min[1]);
		bounding_box_min[2] = min(temp[2], bounding_box_min[2]);
		bounding_box_max[0] = max(temp[0], bounding_box_max[0]);
		bounding_box_max[1] = max(temp[1], bounding_box_max[1]);
		bounding_box_max[2] = max(temp[2], bounding_box_max[2]);
	}
}

void CTriMesh::UpdateBoundingSphere(void)
{
	double r = 0.0;
	double r_temp;
	HCCLMesh::VertexIter v_it, v_end(vertices_end());
	HCCLMesh::Point temp;
	for(v_it = vertices_begin(); v_it != v_end; ++v_it)
	{
		r_temp = point(v_it).norm();
		r = /*std::*/max(r_temp, r);
	}
	bounding_sphere_rad = r;
}

double CTriMesh::GetBoundingSphereRadius(void) const
{
	return bounding_sphere_rad;
}

void CTriMesh::Translate(double x, double y, double z)
{
	std::for_each(vertices_begin(), vertices_end(), [&](const HCCLMesh::VertexHandle& vh){
		this->set_point(vh, point(vh)+ HCCLMesh::Point(x, y, z));
	});
}

void CTriMesh::Translate(Vector3d v)
{
	std::for_each(vertices_begin(), vertices_end(), [&](const HCCLMesh::VertexHandle& vh){
		this->set_point(vh, HCCLMesh::Point(v.X(), v.Y(), v.Z()));
	});
}

void CTriMesh::Translate(OpenMesh::Vec3d v)
{
	std::for_each(vertices_begin(), vertices_end(), [&](const HCCLMesh::VertexHandle& vh){
		this->set_point(vh, v);
	});
}

void CTriMesh::Rotate( double angle, HCCLMesh::Point axis )
{

}

void CTriMesh::Decimate(double p)
{
// 	typedef OpenMesh::Decimater::DecimaterT< HCCLMesh > Decimater;
// 	typedef OpenMesh::Decimater::ModQuadricT< HCCLMesh >::Handle HModQuadric;
// 
// 	Decimater   decimater(*this);  // a decimater object, connected to a mesh
// 	HModQuadric hModQuadric;      // use a quadric module
// 
// 	decimater.add( hModQuadric ); // register module at the decimater
// 
// 	std::cout << decimater.module( hModQuadric ).name() << std::endl;
// 
// 	// the way to access the module 
// 	decimater.initialize();       // let the decimater initialize the mesh and the modules
// 
// 	decimater.decimate_to(n_vertices()*p);
// 
// 	garbage_collection();
}

void CTriMesh::SampleRandom(int nSamples, std::vector<Vector3d>& samples) const
{
	if(n_vertices() <= 0)
		return;

	//////////////////////////////////////////////////////////////////////////
	// Random Sampling Method
	samples.resize(nSamples);

	// Vertices List
	std::vector<int> vtxIdxList;
	std::vector<int> vtxSelect(nSamples);

	vtxIdxList.resize(n_vertices());
	std::iota(vtxIdxList.begin(), vtxIdxList.end(), 0);				// Generate a vertex index list (Non-zero base indexing)

	// Random Shuffle
	std::random_shuffle(vtxIdxList.begin(), vtxIdxList.end());		// Randomly shuffle
	std::copy(vtxIdxList.begin(), vtxIdxList.begin()+nSamples, vtxSelect.begin());
	std::sort(vtxSelect.begin(), vtxSelect.end());

	HCCLMesh::ConstVertexIter vit(vertices_begin()), vEnd(vertices_end());
	HCCLMesh::Point temp;
	int cnt = 0;
	int idxCnt = 0;
	for(; vit != vEnd; ++vit, ++cnt)
	{
		if(cnt==vtxSelect[idxCnt])
		{
			HCCLMesh::Point pt = point(vit);
			samples[idxCnt] = Vector3d(pt[0], pt[1], pt[2]);
		
			idxCnt++;
			if(idxCnt==nSamples)
				break;
		}				
	}
}

void CTriMesh::SampleRandom( int nSamples, std::vector<Vector3d>& samples, std::vector<int>& indx ) const
{

}

void CTriMesh::SampleUniform(int nSamples, std::vector<Vector3d>& samples, uint nFlag/* = TM_SAMPLE_UNIFORM_DART*/) const
{
	int s_tic = clock();
	if( (nFlag & TM_SAMPLE_UNIFORM_DART) == TM_SAMPLE_UNIFORM_DART )
	{
		HCCLMesh::ConstFaceIter fIt(faces_begin()), fEnd(faces_end());
		HCCLMesh::ConstFaceVertexIter fvIt;
		HCCLMesh::Point pt1, pt2, pt3;		
		double area = 0;
		double mean_dist = 0;
		for (; fIt!=fEnd; ++fIt)
		{
			fvIt = cfv_iter(fIt.handle());

			pt1 = point(fvIt.handle());
			pt2 = point((++fvIt).handle());
			pt3 = point((++fvIt).handle());

			HCCLMesh::Point t1 = pt1-pt2;
			HCCLMesh::Point t2 = pt1-pt3;
			area += 0.5*(t1%t2).norm();
			mean_dist += 0.5*(t1.norm()+t2.norm());			
		}	

		mean_dist /= n_vertices();
		SampleRandom(10*nSamples > n_vertices() ? n_vertices() : 10*nSamples, samples);
	
		if(n_vertices() <= 0  || samples.size() == 0)
			return;

		int n_nodes = samples.size();
		double dist = sqrt(area/(double)(sqrt(3.0)*nSamples));
		std::vector<Vector3d> D_nodes(nSamples);
		Vector3d temp;
		auto f1 = [this, &temp, &dist](Vector3d v)->bool
		{
			if((temp - v).Norm() < dist)
				return true;
			else
				return false;
		};

		int cnt = 0;
		int r;
		int num_iter = 0;
		while(1)
		{
			n_nodes = samples.size();
			r = rand()%n_nodes;
			temp = Vector3d(samples[r].X(), samples[r].Y(), samples[r].Z());
			D_nodes[cnt++] = Vector3d(samples[r].X(), samples[r].Y(), samples[r].Z());
			samples.erase(std::remove_if(samples.begin(), samples.end(), f1), samples.end());
			if(cnt < 0.98*nSamples && samples.size() == 0)
			{
				SampleRandom(10*nSamples > n_vertices() ? n_vertices() : 10*nSamples, samples);
				cnt = 0;
				dist *= 0.98;
				num_iter++;
			}
			else if(cnt == nSamples && samples.size() != 0)
			{
				SampleRandom(10*nSamples > n_vertices() ? n_vertices() : 10*nSamples, samples);
				cnt = 0;
				dist *= 1.02;
				num_iter++;
			}
			else if(cnt >= 0.98*nSamples && samples.size() == 0)
				break;
		}

		auto f2 = [](Vector3d v)->bool
		{
			if(v.Norm()==0)
				return true;
			else
				return false;
		};
		D_nodes.erase(std::remove_if(D_nodes.begin(), D_nodes.end(), f2), D_nodes.end());
		samples = D_nodes;

	}
	int e_tic = clock();
	std::cout << (e_tic - s_tic)/(double)CLOCKS_PER_SEC << "sec"<< std::endl;
}

void CTriMesh::SampleUniform( int nSamples, std::vector<Vector3d>& samples, std::vector<int>& indx, uint nFlag /*= TM_SAMPLE_UNIFORM_DART*/ ) const
{
	int s_tic = clock();
	if( (nFlag & TM_SAMPLE_UNIFORM_DART) == TM_SAMPLE_UNIFORM_DART )
	{
		HCCLMesh::ConstFaceIter fIt(faces_begin()), fEnd(faces_end());
		HCCLMesh::ConstFaceVertexIter fvIt;
		HCCLMesh::Point pt1, pt2, pt3;		
		double area = 0;
		double mean_dist = 0;
		for (; fIt!=fEnd; ++fIt)
		{
			fvIt = cfv_iter(fIt.handle());

			pt1 = point(fvIt.handle());
			pt2 = point((++fvIt).handle());
			pt3 = point((++fvIt).handle());

			HCCLMesh::Point t1 = pt1-pt2;
			HCCLMesh::Point t2 = pt1-pt3;
			area += 0.5*(t1%t2).norm();
			mean_dist += 0.5*(t1.norm()+t2.norm());			
		}	

		mean_dist /= n_vertices();
		SampleRandom(10*nSamples > n_vertices() ? n_vertices() : 10*nSamples, samples);

		if(n_vertices() <= 0  || samples.size() == 0)
			return;

		int n_nodes = samples.size();
		double dist = sqrt(area/(double)(sqrt(3.0)*nSamples));
		std::vector<Vector3d> D_nodes(nSamples);
		Vector3d temp;
		auto f1 = [this, &temp, &dist](Vector3d v)->bool
		{
			if((temp - v).Norm() < dist)
				return true;
			else
				return false;
		};

		int cnt = 0;
		int r;
		int num_iter = 0;
		while(1)
		{
			n_nodes = samples.size();
			r = rand()%n_nodes;
			temp = Vector3d(samples[r].X(), samples[r].Y(), samples[r].Z());
			D_nodes[cnt++] = Vector3d(samples[r].X(), samples[r].Y(), samples[r].Z());
			samples.erase(std::remove_if(samples.begin(), samples.end(), f1), samples.end());
			if(cnt < 0.98*nSamples && samples.size() == 0)
			{
				SampleRandom(10*nSamples > n_vertices() ? n_vertices() : 10*nSamples, samples);
				cnt = 0;
				dist *= 0.98;
				num_iter++;
			}
			else if(cnt == nSamples && samples.size() != 0)
			{
				SampleRandom(10*nSamples > n_vertices() ? n_vertices() : 10*nSamples, samples);
				cnt = 0;
				dist *= 1.02;
				num_iter++;
			}
			else if(cnt >= 0.98*nSamples && samples.size() == 0)
				break;
		}

		auto f2 = [](Vector3d v)->bool
		{
			if(v.Norm()==0)
				return true;
			else
				return false;
		};
		D_nodes.erase(std::remove_if(D_nodes.begin(), D_nodes.end(), f2), D_nodes.end());
		samples = D_nodes;
	}

	int e_tic = clock();
	std::cout << (e_tic - s_tic)/(double)CLOCKS_PER_SEC << "sec"<< std::endl;
}

inline double tac( IndexedPoint indexed_pt, size_t k ) { return indexed_pt.first[k]; }
void CTriMesh::BuildKDTree(void)
{
	DestroyKDTree();
	kdtree = new HCCLKDTree(std::ptr_fun(tac));
	std::for_each(vertices_begin(), vertices_end(), [&](const HCCLMesh::VertexHandle& vh){
		kdtree->insert(IndexedPoint(point(vh), vh.idx()));
	});
}
void CTriMesh::FindClosestPoint(Vector3d ref, int* idx, int n/* = 1*/, Vector3d* pt/* = NULL*/) const
{
	struct FindN_predicate
	{
		typedef std::pair<double, IndexedPoint> Candidate;
		typedef std::vector<Candidate> Candidates;

		struct Data
		{
			Data(HCCLMesh::Point t, size_t n) : target(t), num_wanted(n)	{candidates.reserve(n);}
			
			Candidates candidates;
			HCCLMesh::Point target;
			size_t num_wanted;
		};

		FindN_predicate(Data * data_) : data(data_), cs(&data_->candidates) {}

		bool operator()( IndexedPoint const& t )
		{
			if (data->num_wanted > 0)
			{
				double dist = (data->target - t.first).norm();
				bool full = (cs->size() == data->num_wanted);

				if (!full || dist < cs->back().first)
				{
					bool let_libkdtree_trim = false;
					if (full)
					{
						cs->pop_back();
						let_libkdtree_trim = (cs->empty() || dist > cs->back().first);
					}
					cs->insert( std::lower_bound(cs->begin(),cs->end(),/*Candidate(dist,t)*/dist, [&](Candidate& c, const double& d)->bool{return (c.first < d);}), Candidate(dist,t) );
					return let_libkdtree_trim;
				}
			}
			return true;
		}

		Data * data;
		Candidates * cs;
	};

	IndexedPoint target(HCCLMesh::Point(ref[0], ref[1], ref[2]), -1);

	FindN_predicate::Data nearest_n(target.first, n);
	kdtree->find_nearest_if(target, std::numeric_limits<double>::infinity(), FindN_predicate(&nearest_n));

	
	for (int i = 0; i < nearest_n.candidates.size(); ++i)
	{
		idx[i] = nearest_n.candidates[i].second.second;
	}
	if (pt!=NULL)
	{
		for (int i = 0; i < nearest_n.candidates.size(); ++i)
		{
			HCCLMesh::Point hccl_pt = nearest_n.candidates[i].second.first;
			pt[i] = Vector3d(hccl_pt.data());
		}			
	}

}
void CTriMesh::DestroyKDTree(void)
{
	if(kdtree)
	{
		delete kdtree;
		kdtree = NULL;
	}
}


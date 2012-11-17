#include "DeformationGraph.h"
#include "OpenMeshWrapper.h"

#ifdef QT_OPENGL_LIB
#include <qgl.h>
#else
#include <gl/GL.h>
#include <gl/GLU.h>
#endif

#include <algorithm>


DeformationGraph::DeformationGraph(void)
	: mesh(NULL)
	, kdtree(NULL)
{
}

DeformationGraph::DeformationGraph(CTriMesh* _mesh)
	: mesh(NULL)
	, kdtree(NULL)
{
	mesh = _mesh;
}

DeformationGraph::~DeformationGraph(void)
{
	Clear();
}

void DeformationGraph::Clear(void)
{
	DestroyKDTree();
	mesh = NULL;
	nodes.clear();
	edges.clear();
	nodes_neighboring_nodes.clear();
}

void DeformationGraph::SetMesh(CTriMesh* _mesh)
{
	mesh = _mesh;
}

#include <windows.h>
void DeformationGraph::BuildGraph(double sampling_rate/* = 0.2*/, int k/* = 4*/)
{
	LARGE_INTEGER ticksPerSecond;
	LARGE_INTEGER start, end;
	LARGE_INTEGER time;
	DWORD microsec;
	QueryPerformanceFrequency(&ticksPerSecond);

	mesh->SampleUniform(mesh->n_vertices()*sampling_rate, nodes, TM_SAMPLE_UNIFORM_DART);

	mesh->BuildKDTree();
	node_indxes.resize(nodes.size());
	for (int i = 0; i<nodes.size(); ++i)
	{
		int indx;
		mesh->FindClosestPoint(nodes[i], &indx);
		node_indxes[i] = indx;
	}

	printf("Building KD-Tree: ");
	QueryPerformanceCounter(&start);
	BuildKDTree();
	QueryPerformanceCounter(&end);
	microsec = (double)((end.QuadPart-start.QuadPart)/(ticksPerSecond.QuadPart/1000000));
	printf("%d.%dms\n", microsec/1000, microsec%1000);

	printf("Linking nodes sharing effects: ");
	QueryPerformanceCounter(&start);
	int cnt = 0;
	std::vector<int> idx(k);
	std::for_each(mesh->vertices_begin(), mesh->vertices_end(), [&](const HCCLMesh::VertexHandle& vh){
		HCCLMesh::Point pt = mesh->point(vh);
		FindClosestPoint(Vector3d(pt[0], pt[1], pt[2]), &idx[0], k);
		std::sort(idx.begin(), idx.end());
		for(int i = 0; i < k-1; i++)
		{
			for(int j = i+1; j < k; j++)
			{
				edges.push_back(Vector2i(idx[i], idx[j]));
			}
		}
	});
	QueryPerformanceCounter(&end);
	microsec = (double)((end.QuadPart-start.QuadPart)/(ticksPerSecond.QuadPart/1000000));
	printf("%d.%dms\n", microsec/1000, microsec%1000);

	printf("Unique: ");
	QueryPerformanceCounter(&start);
	std::sort(edges.begin(), edges.end(), [&](Vector2i& ei, Vector2i& ej)->bool{
		if(ei[0] < ej[0]) return true;
		else if(ei[0] > ej[0]) return false;
		else return (ei[1] < ej[1]);
	});
	std::vector<Vector2i>::iterator it = std::unique(edges.begin(), edges.end());
	edges.erase(it, edges.end());	
	QueryPerformanceCounter(&end);
	microsec = (double)((end.QuadPart-start.QuadPart)/(ticksPerSecond.QuadPart/1000000));
	printf("%d.%dms\n", microsec/1000, microsec%1000);

	nodes_neighboring_nodes.resize(nodes.size());
	std::for_each(edges.begin(), edges.end(), [&](Vector2i& e){
		nodes_neighboring_nodes[e[0]].push_back(e[1]);
		nodes_neighboring_nodes[e[1]].push_back(e[0]);
	});
}

void DeformationGraph::BuildGraph( const std::vector<std::vector<double>>& dmap, int k /*= 2*/ ) /* void SetDeformationConstraints(something)*/
{
	int y = dmap.size();
	int x = dmap[0].size();

	nodes.reserve(x*y);

	for (int i = 0; i<y; ++i)
	{
		for (int j = 0; j<x; ++j)
		{
			nodes.push_back(Vector3d(j,i,dmap[i][j]));
		}
	}

	BuildKDTree();

// 	int cnt = 0;
// 	std::vector<int> idx(k);
// 	std::for_each(mesh->vertices_begin(), mesh->vertices_end(), [&](const HCCLMesh::VertexHandle& vh){
// 		HCCLMesh::Point pt = mesh->point(vh);
// 		FindClosestPoint(Vector3d(pt[0], pt[1], pt[2]), &idx[0], k);
// 		std::sort(idx.begin(), idx.end());
// 		for(int i = 0; i < k-1; i++)
// 		{
// 			for(int j = i+1; j < k; j++)
// 			{
// 				edges.push_back(Vector2i(idx[i], idx[j]));
// 			}
// 		}
// 	});
// 
// 	std::sort(edges.begin(), edges.end(), [&](Vector2i& ei, Vector2i& ej)->bool{
// 		if(ei[0] < ej[0]) return true;
// 		else if(ei[0] > ej[0]) return false;
// 		else return (ei[1] < ej[1]);
// 	});
// 	std::vector<Vector2i>::iterator it = std::unique(edges.begin(), edges.end());
// 	edges.erase(it, edges.end());	
// 
// 	nodes_neighboring_nodes.resize(nodes.size());
// 	std::for_each(edges.begin(), edges.end(), [&](Vector2i& e){
// 		nodes_neighboring_nodes[e[0]].push_back(e[1]);
// 		nodes_neighboring_nodes[e[1]].push_back(e[0]);
// 	});

}

void DeformationGraph::FindClosestPoint(Vector3d ref, int* idx, int n/* = 1*/, Vector3d* pt/* = NULL*/) const
{
	struct FindN_predicate
	{
		typedef std::pair<double, DG_IndexedPoint> Candidate;
		typedef std::vector<Candidate> Candidates;

		struct Data
		{
			Data(Vector3d t, size_t n) : target(t), num_wanted(n)	{candidates.reserve(n);}

			Candidates candidates;
			Vector3d target;
			size_t num_wanted;
		};

		FindN_predicate(Data * data_) : data(data_), cs(&data_->candidates) {}

		bool operator()( DG_IndexedPoint const& t )
		{
			if (data->num_wanted > 0)
			{
				double dist = (data->target - t.first).Norm();
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
					//cs->insert( lower_bound(cs->begin(),cs->end(),/*Candidate(dist,t)*/dist), Candidate(dist,t) );
					return let_libkdtree_trim;
				}
			}
			return true;
		}

		Data * data;
		Candidates * cs;
	};

	DG_IndexedPoint target(Vector3d(ref[0], ref[1], ref[2]), -1);

	FindN_predicate::Data nearest_n(target.first, n);
	kdtree->find_nearest_if(target, std::numeric_limits<double>::infinity(), FindN_predicate(&nearest_n));

	//std::transform(nearest_n.candidates.begin(), nearest_n.candidates.end(), idx, [&](FindN_predicate::Candidate& c)->int{ return c.second.second; });
	for (int i = 0; i < nearest_n.candidates.size(); ++i)
		idx[i] = nearest_n.candidates[i].second.second;

	if(pt)
	{
		for (int i = 0; i < nearest_n.candidates.size(); ++i)
			pt[i] = nearest_n.candidates[i].second.first;
	}
}

inline double tac( DG_IndexedPoint indexed_pt, size_t k ) { return indexed_pt.first[k]; }
void DeformationGraph::BuildKDTree(void)
{
	kdtree = new DG_KDTree(std::ptr_fun(tac));
	for(size_t i = 0; i < nodes.size(); i++)
		kdtree->insert(DG_IndexedPoint(nodes[i], i));
}
void DeformationGraph::DestroyKDTree(void)
{
	if(kdtree)
	{
		delete kdtree;
		kdtree = NULL;
	}
}

void DeformationGraph::GetNeighbors(int i, std::vector<int>& idx) const
{
	idx = nodes_neighboring_nodes[i];
}

void DeformationGraph::Render(void)
{
	glColor3ub(100,40,10);
	glBegin(GL_POINTS);
	for(int i = 0; i < draw_nodes.size(); i++)
		glVertex3d(draw_nodes[i].X(), draw_nodes[i].Y(), draw_nodes[i].Z());
	glEnd();

	glBegin(GL_LINES);
	for(int i = 0; i < edges.size(); i++)
	{
		glVertex3d(draw_nodes[edges[i][0]].X(), draw_nodes[edges[i][0]].Y(), draw_nodes[edges[i][0]].Z());
		glVertex3d(draw_nodes[edges[i][1]].X(), draw_nodes[edges[i][1]].Y(), draw_nodes[edges[i][1]].Z());
	}
	glEnd();
}
#include "ShortestPath.h"

using namespace cv;
using namespace std;

static float getLineDistance( const cv::Vec4f& l1, const cv::Vec4f& l2 )
{
	Point2f grip1(l1[0], l1[1]);
	Point2f head1(l1[2], l1[3]);
	Point2f grip2(l2[0], l2[1]);
	Point2f head2(l2[2], l2[3]);

// 	Point2f dir1 = head1-grip1;
// 	Point2f dir2 = head2-grip2;
// 	float angl1 = atan2(dir1.y,dir1.x)*180/CV_PI;
// 	if (angl1 < 0) angl1 += 180;
// 	float angl2 = atan2(dir2.y,dir2.x)*180/CV_PI;
// 	if (angl2 < 0) angl2 += 180;
// 	float angle_diff = abs(angl1-angl2);

// 	Point2f center1 = (grip1+head1)*0.5;
// 	Point2f center2 = (grip2+head2)*0.5;

	double head_diff = norm(head2-head1)*0.01;
	double grip_diff = norm(grip2-grip1)*0.01;

	return head_diff*head_diff + grip_diff*grip_diff;
}

float mx = 9999999 ;

ShortestPath::ShortestPath( const std::vector<std::vector<cv::Vec4f>>& data )
{
	indx_size = data.size();

	int i = 0,
		j = 0,
		count = 0;

	nodeVector.reserve(1000);
	for (i = 0; i<data.size(); ++i)
	{
		if (data[i].empty())
		{
			Node tem(i, 0, count);
			++count;
			nodeVector.push_back(tem);
		} 
		else
		{
			for (j = 0; j<data[i].size(); ++j)
			{
				Node tem(i, j, count);
				++count;
				nodeVector.push_back(tem);	
			}
		}
	}
	num_node = count;
	nodeVector.reserve(num_node);
	graphMap.resize(num_node);
	for (i = 0; i<num_node; ++i)
	{
		graphMap[i].resize(num_node, mx);
		graphMap[i][i] = 0;
		for (j = i+1; j<num_node; ++j)
		{
			if (nodeVector[i].m_indx+1 == nodeVector[j].m_indx)
			{
				if (data[nodeVector[i].m_indx].empty() || data[nodeVector[j].m_indx].empty()) graphMap[i][j] = 0;
				else graphMap[i][j] = getLineDistance(data[nodeVector[i].m_indx][nodeVector[i].s_indx], data[nodeVector[j].m_indx][nodeVector[j].s_indx]);
			}			
			else if (nodeVector[i].m_indx+1 < nodeVector[j].m_indx) break;
		}
	}
}

ShortestPath::~ShortestPath(void)
{
}

std::vector<int> ShortestPath::findPath()
{
	const int N = num_node;

	int j, k, p, start;
		
	float min;
	vector<float> leng(N, mx);	// 정점까지 거리
	
	vector<int>	v(N, 0),		// 확정 플래그
		indx(N);				// 이전 정점을 가리키는 포인터


	start = 0;

	// 거리와 플래그 초기화	
	leng[start] = 0;
	indx[start] = 0;


	for (j=0; j<N; j++)
	{
		min = mx;
		// 최단거리 정점 탐색
		for (k = 0; k<N; ++k)
		{
			if (v[k]==0 && leng[k]<min)
			{
				p = k;
				min = leng[k];
			}
		}
		v[p] = 1;

// 		if (min==mx)
// 		{
// 			printf("그래프가 연결되어있지 않다.\n");
// 			return 1;
// 		}


		// p를 경유해서 k에 이르는 거리가 지금까지의 쵠단경로보다 작으면 갱신
		for (k = 0; k<N; ++k)
		{
			if ((leng[p]+graphMap[p][k])<leng[k])
			{
				leng[k] = leng[p] + graphMap[p][k];
				indx[k] = p;
			}
		}
	}

	//최단경로 출력
	vector<int> indx1;
	j = N-1;
	printf("%f : %d\n", leng[j], j);
	p = j;
	indx1.push_back(nodeVector[p].s_indx);
	while(indx[p] != 0)
	{
		indx1.push_back(nodeVector[indx[p]].s_indx);
		p = indx[p];		
	}
	indx1.push_back(nodeVector[indx[p]].s_indx);

	reverse(indx1.begin(), indx1.end());
	return indx1;
}

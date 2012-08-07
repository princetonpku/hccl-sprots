#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class Node
{
public:
	Node(int a, int b, int c)
	{
		m_indx = a;
		s_indx = b;
		node_indx = c;
	};
	~Node(){};

	int m_indx;
	int s_indx;
	int node_indx;
};

class ShortestPath
{
public:
	ShortestPath(const std::vector<std::vector<cv::Vec4f>>& data);
	~ShortestPath(void);

	std::vector<int> findPath();
	
private:

	int indx_size;
	int num_node;

	std::vector<Node> nodeVector;
	std::vector<std::vector<float>> graphMap;
};


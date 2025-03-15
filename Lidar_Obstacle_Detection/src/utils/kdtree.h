#include "render/render.h"

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(const std::vector<float>& arr, int setId)
	: point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	void insertHelper(Node*& node, uint depth, const std::vector<float>& point, int id)
    {
        if (node == nullptr)
            node = new Node(point, id);
        else
        {
            uint cd = depth % 3;
            if (point[cd] < node->point[cd])
                insertHelper(node->left, depth + 1, point, id);
            else
                insertHelper(node->right, depth + 1, point, id);
        }
    }

	void insert(const std::vector<float>& point, int id)
	{
		insertHelper(root, 0, point, id);
	}

	inline bool isInBox(const std::vector<float>& target, const std::vector<float>& point, float distanceTol)
    {
        return (point[0] >= target[0] - distanceTol && point[0] <= target[0] + distanceTol &&
                point[1] >= target[1] - distanceTol && point[1] <= target[1] + distanceTol &&
                point[2] >= target[2] - distanceTol && point[2] <= target[2] + distanceTol);
    }

	inline bool isInDistance(const std::vector<float>& target, const std::vector<float>& point, float distanceTol)
    {
        float distSq = (target[0] - point[0]) * (target[0] - point[0]) +
                       (target[1] - point[1]) * (target[1] - point[1]) +
                       (target[2] - point[2]) * (target[2] - point[2]);
        return distSq <= distanceTol * distanceTol;
    }

	void searchHelper(const std::vector<float>& target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != nullptr)
		{
			if (isInBox(target, node->point, distanceTol) && isInDistance(target, node->point, distanceTol))
			{
				ids.push_back(node->id);
			}

			uint cd = depth % 3;
			if (target[cd] - distanceTol < node->point[cd])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if (target[cd] + distanceTol >= node->point[cd])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}

	std::vector<int> search(const std::vector<float>& target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};

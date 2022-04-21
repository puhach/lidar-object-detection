
//#include "../../render/render.h"

// TODO: consider making it a nested struct of KdTree
// A structure representing a node of the k-d tree
template <typename T>
struct Node
{
	Node(std::size_t index, const T& data) : index(index), data(data) {}
	Node(std::size_t index, T&& data) : index(index), data(std::move(data)) {}
	
	std::size_t index;
	T data;
	std::unique_ptr<Node> left, right;
};	// Node


// The default data adaptor for making arbitrary data structures with X, Y, Z fields indexable
template <typename T>
struct XYZAdaptor
{
	constexpr XYZAdaptor(const T& data) : data(data) {}
	constexpr XYZAdaptor(T&& data) : data(std::move(data)) {}

	static constexpr std::size_t dim() noexcept { return 3; }

	constexpr decltype(auto) operator [](std::size_t index) const
	{
		switch (index)
		{
		case 0: return data.x;
		case 1: return data.y;
		case 2: return data.z;
		default: throw std::out_of_range("Invalid coordinate index.");
		}
	}	// operator []

	T data;
};	// XYZAdaptor


template <typename Data, typename Item = XYZAdaptor<Data>>
//template <typename Data, typename Item = Data>
class KdTree
{
public:
	void insert(std::size_t index, const Data& data) { insertHelper(this->root, index, Item{data}, 0); }

	void insert(std::size_t index, Data&& data) { insertHelper(this->root, index, Item{std::move(data)}, 0); }

	template <typename Distance>
	std::vector<std::size_t> search(const Data& data, Distance distanceTolerance) const;

	void clear() { root.reset(); }

private:

	void insertHelper(std::unique_ptr<Node<Item>>& parent, std::size_t index, Item&& data, std::size_t depth);

	template <typename Distance>
	//void searchHelper(const std::unique_ptr<Node<Item>> &parent, const Item& target, Distance distanceTolerance, std::size_t depth,
	void searchHelper(const Node<Item> *parent, const Item &target, Distance distanceTolerance, std::size_t depth, 
		std::vector<std::size_t> &neighborIndices) const;

	/*
	// My helper
	void insertHelper(Node*& parent, std::vector<float>&& point, int id, int depth)
	{
		
	}	// insertHelper

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(this->root, std::move(point), id, 0);
	}


	void searchHelper(const Node* node, const std::vector<float>& target, float distanceTol, int depth, std::vector<int>& ids)
	{
		if (!node)
			return;

		float d[] = { target[0] - node->point[0], target[1] - node->point[1] };

		if (d[0] * d[0] + d[1] * d[1] <= distanceTol * distanceTol)	// inside the sphere
			ids.push_back(node->id);
		
		int dim = depth % target.size();
		int targetCoord = target[dim];
		int nodeCoord = node->point[dim];

		// Children with a smaller coord may be inside the box
		// nodeCoord > targetCoord - distanceTol => distanceTol > targetCoord - nodeCoord =>
		// distanceTol > d[dim] => d[dim] < distanceTol
		if (d[dim] < distanceTol)	
			searchHelper(node->left, target, distanceTol, depth + 1, ids);

		// Children with a greater or equal coord may be inside the box
		// nodeCoord <= targetCoord + distanceTol => nodeCoord - targetCoord <= distanceTol =>
		// targetCoord - nodeCoord >= -distanceTol => d[dim] >= -distanceTol
		if (d[dim] >= -distanceTol)	
			searchHelper(node->right, target, distanceTol, depth + 1, ids);

	}	// searchHelper

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(this->root, target, distanceTol, 0, ids);
		return ids;
	}
	*/

	std::unique_ptr<Node<Item>> root;
};	// KdTree


// TODO: consider using perfrect forwarding
template<typename Data, typename Item>
inline void KdTree<Data, Item>::insertHelper(std::unique_ptr<Node<Item>>& parent, std::size_t index, Item&& data, std::size_t depth)
{
	if (parent)
	{
		auto axis = depth % data.dim();
		decltype(auto) newCoord = data[axis];
		decltype(auto) parentCoord = parent->data[axis];

		if (newCoord < parentCoord)
			insertHelper(parent->left, index, std::move(data), depth + 1);
		else
			insertHelper(parent->right, index, std::move(data), depth + 1);
	}
	else
	{
		parent = std::make_unique<Node<Item>>(index, data);
	}
}	// insertHelper



template <typename Data, typename Item>
template <typename Distance>
std::vector<std::size_t> KdTree<Data, Item>::search(const Data& target, Distance distanceTolerance) const
{
	std::vector<std::size_t> neighborIndices;
	searchHelper(this->root.get(), target, distanceTolerance, 0, neighborIndices);
	//searchHelper(this->root, target, distanceTolerance, 0, neighborIndices);
	return neighborIndices;
}	// search

template<typename Data, typename Item>
template<typename Distance>
void KdTree<Data, Item>::searchHelper(const Node<Item>* parent, const Item& target, Distance distanceTolerance, std::size_t depth,
//void KdTree<Data, Item>::searchHelper(const std::unique_ptr<Node<Item>> &parent, const Item& target, Distance distanceTolerance, std::size_t depth,
	std::vector<std::size_t>& neighborIndices) const
{
	if (!parent)
		return;

	Distance dist2{};	// zero initialize the distance
	for (std::size_t i = 0; i < target.dim(); ++i)
	{
		decltype(auto) diff = target[i] - parent->data[i];
		dist2 += diff * diff;
	}

	if (dist2 <= distanceTolerance * distanceTolerance)
		neighborIndices.push_back(parent->index);

	auto axis = depth % target.dim();
	decltype(auto) targetCoord = target[axis];
	decltype(auto) parentCoord = parent->data[axis];

	// Children with a smaller coord may be inside the box
	if (parentCoord > targetCoord - distanceTolerance)
		searchHelper(parent->left.get(), target, distanceTolerance, depth + 1, neighborIndices);
	
	// Children with a greater or equal coord may be inside the box
	if (parentCoord <= targetCoord + distanceTolerance)
		searchHelper(parent->right.get(), target, distanceTolerance, depth + 1, neighborIndices);

	/*std::array<Distance, Item::dim()> d;
	for (std::size_t i = 0; i < d.size(); ++i)
	{
		d[i] = target[i] - 
	}*/
	//float d[] = { target[0] - node->point[0], target[1] - node->point[1] };

	//if (d[0] * d[0] + d[1] * d[1] <= distanceTol * distanceTol)	// inside the sphere
	//	ids.push_back(node->id);

	/*
	int dim = depth % target.size();
	int targetCoord = target[dim];
	int nodeCoord = node->point[dim];

	// Children with a smaller coord may be inside the box
	// nodeCoord > targetCoord - distanceTol => distanceTol > targetCoord - nodeCoord =>
	// distanceTol > d[dim] => d[dim] < distanceTol
	if (d[dim] < distanceTol)
		searchHelper(node->left, target, distanceTol, depth + 1, ids);

	// Children with a greater or equal coord may be inside the box
	// nodeCoord <= targetCoord + distanceTol => nodeCoord - targetCoord <= distanceTol =>
	// targetCoord - nodeCoord >= -distanceTol => d[dim] >= -distanceTol
	if (d[dim] >= -distanceTol)
		searchHelper(node->right, target, distanceTol, depth + 1, ids);
		*/
}	// searchHelper



//#include "../../render/render.h"

// TODO: consider making it a nested struct of KdTree
// A structure representing a node of the k-d tree
template <typename T>
struct Node
{
	//Node(std::size_t index, const T& data) : index(index), data(data) {}
	//Node(std::size_t index, T&& data) : index(index), data(std::move(data)) {}
	Node(const T& data) : data(data) {}
	Node(T&& data) : data(std::move(data)) {}

	//std::size_t index;
	T data;
	std::unique_ptr<Node> left, right;
};	// Node


/*
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
*/

//template <typename Data, typename Item = XYZAdaptor<Data>>
//template <typename Data, typename Item = Data>
template <typename Item>
class KdTree
{
public:
	void insert(const Item& data) { insertHelper(this->root, data, 0); }

	void insert(Item&& data) { insertHelper(this->root, std::move(data), 0); }

	template <typename Distance>
	std::vector<Item> search(const Item& data, Distance distanceTolerance) const;
	//std::vector<std::size_t> search(const Item& data, Distance distanceTolerance) const;

	void clear() { root.reset(); }

private:

	template <typename ItemT>
	//void insertHelper(std::unique_ptr<Node<Item>>& parent, Item&& data, std::size_t depth);
	void insertHelper(std::unique_ptr<Node<Item>>& parent, ItemT&& data, std::size_t depth);


	template <typename Distance>
	//void searchHelper(const std::unique_ptr<Node<Item>> &parent, const Item& target, Distance distanceTolerance, std::size_t depth,
	void searchHelper(const Node<Item> *parent, const Item &target, Distance distanceTolerance, std::size_t depth, 
		std::vector<Item> &neighbors) const;

	std::unique_ptr<Node<Item>> root;
};	// KdTree


// TODO: consider using perfrect forwarding
template <typename Item>
template <typename ItemT>
inline void KdTree<Item>::insertHelper(std::unique_ptr<Node<Item>>& parent, ItemT&& data, std::size_t depth)
{
	if (parent)
	{
		auto axis = depth % data.dim();
		decltype(auto) newCoord = data[axis];
		decltype(auto) parentCoord = parent->data[axis];

		if (newCoord < parentCoord)
			insertHelper(parent->left, std::move(data), depth + 1);
		else
			insertHelper(parent->right, std::move(data), depth + 1);
	}
	else
	{
		parent = std::make_unique<Node<Item>>(std::forward<ItemT>(data));
	}
}	// insertHelper



template <typename Item>
template <typename Distance>
std::vector<Item> KdTree<Item>::search(const Item& target, Distance distanceTolerance) const
//std::vector<std::size_t> KdTree<Item>::search(const Item& target, Distance distanceTolerance) const
{
	std::vector<Item> neighbors;
	searchHelper(this->root.get(), target, distanceTolerance, 0, neighbors);
	//searchHelper(this->root, target, distanceTolerance, 0, neighborIndices);
	return neighbors;
}	// search

template<typename Item>
template<typename Distance>
void KdTree<Item>::searchHelper(const Node<Item>* parent, const Item& target, Distance distanceTolerance, std::size_t depth,
//void KdTree<Data, Item>::searchHelper(const std::unique_ptr<Node<Item>> &parent, const Item& target, Distance distanceTolerance, std::size_t depth,
	std::vector<Item>& neighbors) const
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
		neighbors.push_back(parent->data);
		//neighborIndices.push_back(parent->index);

	auto axis = depth % target.dim();
	decltype(auto) targetCoord = target[axis];
	decltype(auto) parentCoord = parent->data[axis];

	// Children with a smaller coord may be inside the box
	if (parentCoord > targetCoord - distanceTolerance)
		searchHelper(parent->left.get(), target, distanceTolerance, depth + 1, neighbors);
	
	// Children with a greater or equal coord may be inside the box
	if (parentCoord <= targetCoord + distanceTolerance)
		searchHelper(parent->right.get(), target, distanceTolerance, depth + 1, neighbors);

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


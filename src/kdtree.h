#ifndef KDTREE_H
#define KDTREE_H

// A space-partitioning data structure for organizing points in a k-dimensional space
template <typename Item>
class KdTree
{
public:
	void insert(const Item& data) { insertHelper(this->root, data, 0); }

	void insert(Item&& data) { insertHelper(this->root, std::move(data), 0); }

	template <typename Distance>
	std::vector<Item> search(const Item& data, Distance distanceTolerance) const;

	void clear() { root.reset(); }

private:

	// A structure representing a node of the k-d tree
	template <typename T>
	struct Node
	{
		Node(const T& data) : data(data) {}
		Node(T&& data) : data(std::move(data)) {}

		T data;
		std::unique_ptr<Node> left, right;
	};	// Node


	template <typename ItemT>
	void insertHelper(std::unique_ptr<Node<Item>>& parent, ItemT&& data, unsigned int depth);


	template <typename Distance>
	void searchHelper(const Node<Item> *parent, const Item &target, Distance distanceTolerance, unsigned int depth, 
						std::vector<Item> &neighbors) const;

	std::unique_ptr<Node<Item>> root;
};	// KdTree


template <typename Item>
template <typename ItemT>
inline void KdTree<Item>::insertHelper(std::unique_ptr<Node<Item>>& parent, ItemT&& data, unsigned int depth)
{
	if (parent)
	{
		auto axis = depth % data.size();
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
{
	std::vector<Item> neighbors;
	searchHelper(this->root.get(), target, distanceTolerance, 0, neighbors);
	return neighbors;
}	// search

template<typename Item>
template<typename Distance>
void KdTree<Item>::searchHelper(const Node<Item>* parent, const Item& target, Distance distanceTolerance, unsigned int depth,
								std::vector<Item>& neighbors) const
{
	if (!parent)
		return;

	Distance dist2{};	// zero initialize the distance
	for (std::size_t i = 0; i < target.size(); ++i)
	{
		decltype(auto) diff = target[i] - parent->data[i];
		dist2 += diff * diff;
	}

	if (dist2 <= distanceTolerance * distanceTolerance)
		neighbors.push_back(parent->data);

	auto axis = depth % target.size();
	decltype(auto) targetCoord = target[axis];
	decltype(auto) parentCoord = parent->data[axis];

	// Children with a smaller coord may be inside the box
	if (parentCoord > targetCoord - distanceTolerance)
		searchHelper(parent->left.get(), target, distanceTolerance, depth + 1, neighbors);
	
	// Children with a greater or equal coord may be inside the box
	if (parentCoord <= targetCoord + distanceTolerance)
		searchHelper(parent->right.get(), target, distanceTolerance, depth + 1, neighbors);

}	// searchHelper

#endif	// KDTREE_H
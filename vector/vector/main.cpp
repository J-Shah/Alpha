#include<iostream>
#include<vector>
#include<array>

using namespace std;
void printVector(vector<int> b)
	// this prints vector
{
	for (int i = 0; i < b.size(); i++)
	{
		std::cout << "vector[" << i << "] " << b[i] << std::endl;
		std::cout << "size of the vector " << b.size() << std::endl;
	}
}

int main()
{
	std::vector<int> items{ 10,20 };
	std::cout << items[0] << std::endl;

	items.push_back(7);  // add 7 in the vector
	std::cout << items.front() << std::endl;
	std::cout << items.back() << std::endl;
	std::cout << items.at(2) << std::endl; // value at index 2 which is 7 in this case
	printVector(items);

	// Array examples
	cout << "Array examples" << endl;
	constexpr int def_node = 10;

	array<int, def_node> nodes{};
	//cout<< nodes.size<<endl;
	cout << nodes.at(2) << endl;
	std::array<int , static_cast<std::size_t>(def_node)> node;
	cout << std::size_t(def_node) << endl;
	cout << endl << endl;
		std::vector<int> v = { 0, 1, 2, 3, 4, 5 };

		for (const int& i : v) // access by const reference
			std::cout << i << ' ';
			std::cout << '\n';

		for (auto i : v) // access by value, the type of i is int
			std::cout << i << ' ';
			std::cout << '\n';

		for (auto&& i : v) // access by forwarding reference, the type of i is int&
			std::cout << i << ' ';
		std::cout << '\n';

		const auto& cv = v;

		for (auto&& i : cv) // access by f-d reference, the type of i is const int&
			std::cout << i << ' ';
		std::cout << '\n';

		for (int n : {0, 1, 2, 3, 4, 5}) // the initializer may be a braced-init-list
			std::cout << n << ' ';
		std::cout << '\n';

		int a[] = { 0, 1, 2, 3, 4, 5 };
		for (int n : a) // the initializer may be an array
			std::cout << n << ' ';
		std::cout << '\n';
		/*
		for ([[maybe_unused]] int n : a)
			std::cout << 1 << ' '; // the loop variable need not be used
		std::cout << '\n';

		for (auto n = v.size(); int i : v) // the init-statement (C++20)
			std::cout << --n + i << ' ';
		std::cout << '\n';
		*/

	
}

#include<iostream>
#include<vector>

using namespace std;
void printVector(vector<int> b)
{
	for (int i = 0; i < b.size(); i++)
	{
		std::cout << "vector[" << i << "] " << b[i] << std::endl;
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
}
#include <iostream>
#include <string>
#include "include/linkedlist.h"

using namespace std;

void unit_test(int test_no, string test_name, int ans, int res)
{
    if (ans == res) cout << "[CLEAR] Test " << test_no << " : " << test_name << endl;
    else cout << "[     ] Test " << test_no << " : " << test_name << endl;
}

int main()
{
    // Declare and initialize list structure
    MyLinkedList my_list;

    int test_clear = 0;
    int test_total = 0;
    
    unit_test(1, "append return 1", my_list.append(2), 0);
    unit_test(2, "append return 2", my_list.append(-1), -1);
    unit_test(3, "append & at function 1", my_list.at(0), 2);
    
    my_list.append(8);
    
    unit_test(4, "append & at function 2", my_list.at(1), 8);

    unit_test(5, "insert return 1", my_list.insert(7, 1), 0);
    unit_test(6, "insert return 2", my_list.insert(-1, 1), -1);
    unit_test(7, "insert return 3", my_list.insert(0, 4), -1);
    unit_test(8, "insert return 4", my_list.insert(0, -1), -1);
    unit_test(9, "insert & at function 1", my_list.at(1), 7);
    unit_test(10, "insert & at function 2", my_list.at(2), 8);

    unit_test(11, "max return 1", my_list.max(), 8);

    unit_test(12, "pop return 1", my_list.pop(1), 7);
    unit_test(13, "pop return 2", my_list.pop(4), -1);
    unit_test(14, "pop return 3", my_list.pop(-1), -1);
    unit_test(15, "pop function 1", my_list.at(1), 8);
    
    my_list.pop(0);
    my_list.pop(0);
    
    unit_test(16, "pop function 2", my_list.at(0), -1);
    unit_test(17, "max return 2", my_list.max(), -1);
    unit_test(18, "pop return 4", my_list.pop(0), -1);

    unit_test(19, "pop memory free 1", global_node_count, 1);

    return 0;
}

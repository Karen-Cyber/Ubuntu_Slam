#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>
using namespace std;

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "usage: plotTrajectory path_to_poses\n";
        return -1;
    }
    string filePath = argv[1];
    filePath += "/trajectory.txt";
}
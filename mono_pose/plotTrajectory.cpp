#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>
using namespace std;

void drawTrajectory(vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>);

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "usage: plotTrajectory path_to_poses\n";
        return -1;
    }
    string filePath = argv[1];
    filePath += "/transformt.txt";
    ifstream fin(filePath);
    if (!fin)
    {
        cout << "Error opening the file\n";
        return -1;
    }

    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    while (!fin.eof())
    {
        Eigen::Isometry3d Twr;
        double data;
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                fin >> data;
                Twr(i, j) = data;
            }
        }
        poses.push_back(Twr);
        cout << "Transformation:\n" << Twr.matrix() << endl;
        cout << "-------------------------\n";
    }

    drawTrajectory(poses);
    return 0;
}

void drawTrajectory(vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses)
{
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1 ,1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glLineWidth(2);

        for (size_t i = 0; i < poses.size(); ++i)
        {
            // 画每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses[i].translation();
            Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
            Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
            Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }

        // 画出连线
        for (size_t i = 0; i < poses.size(); i++) 
        {
            glColor3f(1.0, 1.0, 1.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
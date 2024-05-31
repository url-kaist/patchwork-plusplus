#include <patchwork/patchworkpp.h>

#include <iostream>
#include <fstream>
#include <open3d/Open3D.h>

// for list folder
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

using namespace open3d;

int filename_length = std::string("demo_visaulize_copied.cpp").length();
std::string file_dir = std::string(__FILE__);
std::string data_dir = file_dir.substr(0, file_dir.size()-filename_length) + "../../data/";


void read_bin(std::string bin_path, Eigen::MatrixXf &cloud)
{
  FILE *file = fopen(bin_path.c_str(), "rb");
  if (!file) {
    std::cerr << "error: failed to load " << bin_path << std::endl;
    return;
  }

  std::vector<float> buffer(1000000);
  size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;

  cloud.resize(num_points, 4);
  for (int i=0; i<num_points; i++)
  {
    cloud.row(i) << buffer[i*4], buffer[i*4+1], buffer[i*4+2], buffer[i*4+3];
  }
}

void eigen2geo(Eigen::MatrixXf add, std::shared_ptr<geometry::PointCloud> geo)
{
  for ( int i=0; i<add.rows(); i++ ) {
    geo->points_.push_back(Eigen::Vector3d(add.row(i)(0), add.row(i)(1), add.row(i)(2)));
  }
}

void addNormals(Eigen::MatrixXf normals, std::shared_ptr<geometry::PointCloud> geo)
{
  for (int i=0; i<normals.rows(); i++) {
    geo->normals_.push_back(Eigen::Vector3d(normals.row(i)(0), normals.row(i)(1), normals.row(i)(2)));
  }
}


int main(int argc, char* argv[]) {

  cout << "Execute" << __FILE__ << endl;
  // Get the dataset
  std::string input_cloud_filepath;
  if (argc < 2) {
    // Try out running on the test datasets.
    input_cloud_filepath = data_dir + "000000.bin";
    std::cout << "\033[1;33mNo point cloud file path specified; defaulting to the test directory. \033[0m" << std::endl;
  } else {
    input_cloud_filepath = argv[1];
    std::cout << "\033[1;32mLoading point cloud files from " << input_cloud_filepath << "\033[0m" << std::endl;
  }
  if(!fs::exists(input_cloud_filepath)){
    std::cout << "\033[1;31mERROR HERE: maybe wrong data file path, please check the path or remove argv to run default one. \033[0m" 
              << "\nThe file path you provide is: " << input_cloud_filepath << std::endl;
    return 0;
  }

  // Patchwork++ initialization
  patchwork::Params patchwork_parameters;
  patchwork_parameters.verbose = true;

  patchwork::PatchWorkpp Patchworkpp(patchwork_parameters);

  // Load point cloud
  Eigen::MatrixXf cloud;
  read_bin(input_cloud_filepath, cloud);

  // Estimate Ground
  Patchworkpp.estimateGround(cloud);
  
  // Get Ground and Nonground
  Eigen::MatrixX3f ground     = Patchworkpp.getGround();
  Eigen::MatrixX3f nonground  = Patchworkpp.getNonground();
  double time_taken = Patchworkpp.getTimeTaken();

  Eigen::VectorXi ground_idx    = Patchworkpp.getGroundIndices();
  Eigen::VectorXi nonground_idx = Patchworkpp.getNongroundIndices();

  // Get centers and normals for patches
  Eigen::MatrixX3f centers    = Patchworkpp.getCenters();
  Eigen::MatrixX3f normals    = Patchworkpp.getNormals();

  cout << "Origianl Points  #: " << cloud.rows() << endl;
  cout << "Ground Points    #: " << ground.rows() << endl;
  cout << "Nonground Points #: " << nonground.rows() << endl;
  cout << "Time Taken : "<< time_taken / 1000000 << "(sec)" << endl;
  cout << "Press ... \n" << endl;
  cout << "\t H  : help" << endl;
  cout << "\t N  : visualize the surface normals" << endl;
  cout << "\tESC : close the Open3D window" << endl;

  // Visualize
  std::shared_ptr<geometry::PointCloud> geo_ground(new geometry::PointCloud);
  std::shared_ptr<geometry::PointCloud> geo_nonground(new geometry::PointCloud);
  std::shared_ptr<geometry::PointCloud> geo_centers(new geometry::PointCloud);

  eigen2geo(ground,     geo_ground);
  eigen2geo(nonground,  geo_nonground);
  eigen2geo(centers,    geo_centers);
  addNormals(normals,   geo_centers);

  geo_ground->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
  geo_nonground->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  geo_centers->PaintUniformColor(Eigen::Vector3d(1.0, 1.0, 0.0));  

  visualization::Visualizer visualizer;
  visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
  visualizer.AddGeometry(geo_ground);
  visualizer.AddGeometry(geo_nonground);
  visualizer.AddGeometry(geo_centers);
  visualizer.Run();
  visualizer.DestroyVisualizerWindow();
}
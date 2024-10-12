#ifndef PATCHWORKPP_H
#define PATCHWORKPP_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <numeric>
#include <time.h>

using namespace std;

#define MAX_POINTS 5000

namespace patchwork {

struct PointXYZ {
    float x;
    float y;
    float z;
    int idx;

    PointXYZ(float _x, float _y, float _z, int _idx=-1) : x(_x), y(_y), z(_z), idx(_idx) {}
};

struct RevertCandidate 
{   
    int concentric_idx;
    int sector_idx;
    double ground_flatness;
    double line_variable;
    Eigen::VectorXf pc_mean;
    vector<PointXYZ> regionwise_ground;
    
    RevertCandidate(int _c_idx, int _s_idx, double _flatness, double _line_var, Eigen::VectorXf _pc_mean, vector<PointXYZ> _ground)
     : concentric_idx(_c_idx), sector_idx(_s_idx), ground_flatness(_flatness), line_variable(_line_var), pc_mean(_pc_mean), regionwise_ground(_ground) {}
};

struct Params 
{
    bool verbose;
    bool enable_RNR;
    bool enable_RVPF;
    bool enable_TGR;

    int num_iter;
    int num_lpr;
    int num_min_pts;
    int num_zones;
    int num_rings_of_interest;

    double RNR_ver_angle_thr;
    double RNR_intensity_thr;

    double sensor_height;
    double th_seeds;
    double th_dist;
    double th_seeds_v;
    double th_dist_v;
    double max_range;
    double min_range;
    double uprightness_thr;
    double adaptive_seed_selection_margin;
    double intensity_thr;

    vector<int> num_sectors_each_zone;
    vector<int> num_rings_each_zone;
    
    int max_flatness_storage;
    int max_elevation_storage;

    vector<double> elevation_thr;
    vector<double> flatness_thr;

    
    Params() {
        verbose     = false;
        enable_RNR  = true;
        enable_RVPF = true;
        enable_TGR  = true;

        num_iter = 3;               // Number of iterations for ground plane estimation using PCA.
        num_lpr = 20;               // Maximum number of points to be selected as lowest points representative.
        num_min_pts = 10;           // Minimum number of points to be estimated as ground plane in each patch.
        num_zones = 4;              // Setting of Concentric Zone Model(CZM)
        num_rings_of_interest = 4;  // Number of rings to be checked with elevation and flatness values.

        RNR_ver_angle_thr = -15.0;  // Noise points vertical angle threshold. Downward rays of LiDAR are more likely to generate severe noise points.
        RNR_intensity_thr = 0.2;    // Noise points intensity threshold. The reflected points have relatively small intensity than others.
        
        sensor_height = 1.723;      
        th_seeds = 0.125;           // threshold for lowest point representatives using in initial seeds selection of ground points.
        th_dist = 0.125;            // threshold for thickenss of ground.
        th_seeds_v = 0.25;          // threshold for lowest point representatives using in initial seeds selection of vertical structural points.
        th_dist_v = 0.1;            // threshold for thickenss of vertical structure.
        max_range = 80.0;           // max_range of ground estimation area
        min_range = 2.7;            // min_range of ground estimation area
        uprightness_thr = 0.707;    // threshold of uprightness using in Ground Likelihood Estimation(GLE). Please refer paper for more information about GLE.
        adaptive_seed_selection_margin = -1.2; // parameter using in initial seeds selection

        num_sectors_each_zone = {16, 32, 54, 32};   // Setting of Concentric Zone Model(CZM)
        num_rings_each_zone = {2, 4, 4, 4};         // Setting of Concentric Zone Model(CZM)

        max_flatness_storage = 1000;    // The maximum number of flatness storage
        max_elevation_storage = 1000;   // The maximum number of elevation storage
        elevation_thr = {0, 0, 0, 0};   // threshold of elevation for each ring using in GLE. Those values are updated adaptively.
        flatness_thr = {0, 0, 0, 0};    // threshold of flatness for each ring using in GLE. Those values are updated adaptively.
    }
};

class PatchWorkpp {

public:
    typedef std::vector<vector<PointXYZ>> Ring;
    typedef std::vector<Ring> Zone;

    PatchWorkpp(patchwork::Params _params) : params_(_params) {

        double min_range_z2_ = (7 * params_.min_range + params_.max_range) / 8.0;
        double min_range_z3_ = (3 * params_.min_range + params_.max_range) / 4.0;
        double min_range_z4_ = (params_.min_range + params_.max_range) / 2.0;
        min_ranges_ = {params_.min_range, min_range_z2_, min_range_z3_, min_range_z4_};

        ring_sizes_ = {(min_range_z2_ - params_.min_range) / params_.num_rings_each_zone.at(0),
                      (min_range_z3_ - min_range_z2_) / params_.num_rings_each_zone.at(1),
                      (min_range_z4_ - min_range_z3_) / params_.num_rings_each_zone.at(2),
                      (params_.max_range - min_range_z4_) / params_.num_rings_each_zone.at(3)};
        sector_sizes_ = {2 * M_PI / params_.num_sectors_each_zone.at(0),
                         2 * M_PI / params_.num_sectors_each_zone.at(1),
                         2 * M_PI / params_.num_sectors_each_zone.at(2),
                         2 * M_PI / params_.num_sectors_each_zone.at(3)};

        for (int k = 0; k < params_.num_zones; k++) {
            
            Ring empty_ring;
            empty_ring.resize(params_.num_sectors_each_zone[k]);

            Zone z;
            for (int i = 0; i < params_.num_rings_each_zone[k]; i++) {
                z.push_back(empty_ring);
            }

            ConcentricZoneModel_.push_back(z);
        }

        std::cout << "PatchWorkpp::PatchWorkpp() - INITIALIZATION COMPLETE" << std::endl;
    }

    void estimateGround(Eigen::MatrixXf cloud_in);

    double getHeight() { return params_.sensor_height; }
    double getTimeTaken() { return time_taken_; }

    Eigen::MatrixX3f getGround() { return toEigenCloud(cloud_ground_); }
    Eigen::MatrixX3f getNonground() { return toEigenCloud(cloud_nonground_); }
    Eigen::VectorXi getGroundIndices() { return toIndices(cloud_ground_); }
    Eigen::VectorXi getNongroundIndices() { return toIndices(cloud_nonground_); }
    
    Eigen::MatrixX3f getCenters() { return toEigenCloud(centers_); }
    Eigen::MatrixX3f getNormals() { return toEigenCloud(normals_); }

private:

    // Every private member variable is written with the undescore("_") in its end.

    patchwork::Params params_;

    time_t timer_;
    long time_taken_;

    std::vector<double> update_flatness_[4];
    std::vector<double> update_elevation_[4];

    double d_;

    Eigen::VectorXf normal_;
    Eigen::VectorXf singular_values_;
    Eigen::Matrix3f cov_;
    Eigen::VectorXf pc_mean_;

    vector<double> min_ranges_;
    vector<double> sector_sizes_;
    vector<double> ring_sizes_;

    vector<Zone> ConcentricZoneModel_;

    vector<PointXYZ> ground_pc_, non_ground_pc_;
    vector<PointXYZ> regionwise_ground_, regionwise_nonground_;

    vector<PointXYZ> cloud_ground_, cloud_nonground_;

    vector<PointXYZ> centers_, normals_;

    Eigen::MatrixX3f toEigenCloud(const vector<PointXYZ> &cloud);
    Eigen::VectorXi toIndices(const vector<PointXYZ> &cloud);

    void addCloud(vector<PointXYZ> &cloud, vector<PointXYZ> &add);
    
    void flush_patches(std::vector<Zone> &czm);

    void pc2czm(const Eigen::MatrixXf &src, std::vector<Zone> &czm);

    void reflected_noise_removal(Eigen::MatrixXf &cloud_in);
    
    void temporal_ground_revert(std::vector<double> ring_flatness, std::vector<patchwork::RevertCandidate> candidates, int concentric_idx);
    
    double calc_point_to_plane_d(PointXYZ p, Eigen::VectorXf normal, double d);
    void calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev);

    void update_elevation_thr();
    void update_flatness_thr();

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    void estimate_plane(const vector<PointXYZ> &ground);

    void extract_piecewiseground(
            const int zone_idx, const vector<PointXYZ> &src,
            vector<PointXYZ> &dst,
            vector<PointXYZ> &non_ground_dst);

    void extract_initial_seeds(
            const int zone_idx, const vector<PointXYZ> &p_sorted,
            vector<PointXYZ> &init_seeds);

    void extract_initial_seeds(
            const int zone_idx, const vector<PointXYZ> &p_sorted,
            vector<PointXYZ> &init_seeds, double th_seed);

};

};

#endif

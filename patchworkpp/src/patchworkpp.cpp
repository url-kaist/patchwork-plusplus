#include "patchworkpp.h"

using namespace std;
using namespace patchwork;

bool point_z_cmp(PointXYZ a, PointXYZ b) { return a.z < b.z; }

Eigen::MatrixX3f PatchWorkpp::toEigenCloud(vector<PointXYZ> cloud)
{
    Eigen::MatrixX3f dst(cloud.size(), 3);
    int j=0;
    for (auto &p: cloud) {
        dst.row(j++) << p.x, p.y, p.z;
    }
    return dst;
}

Eigen::MatrixX4f PatchWorkpp::toEigenX4f(vector<PointXYZ> cloud)
{
    Eigen::MatrixX4f dst(cloud.size(), 4);
    int j=0;
    for (auto &p: cloud) {
        dst.row(j++) << p.x, p.y, p.z, p.intensity;
    }
    return dst;
}

void PatchWorkpp::addCloud(vector<PointXYZ> &cloud, vector<PointXYZ> &add)
{
    cloud.insert(cloud.end(), add.begin(), add.end());
}

void PatchWorkpp::flush_patches(vector<Zone> &czm) {

    for (int k = 0; k < params_.num_zones; k++) {
        for (int i = 0; i < params_.num_rings_each_zone[k]; i++) {
            for (int j = 0; j < params_.num_sectors_each_zone[k]; j++) {
                // czm[k][i][j].resize(MAX_POINTS, 3);
                czm[k][i][j].clear();
            }
        }
    }

    if( params_.verbose ) cout << "\033[1;31m" << "PatchWorkpp::flush_patches() - Flushed patches successfully!" << "\033[0m" << endl;
}

void PatchWorkpp::estimate_plane(const vector<PointXYZ> &ground) {

    if (ground.empty()) return;

    Eigen::MatrixX3f eigen_ground(ground.size(), 3);
    int j=0;
    for (auto &p: ground) {
        eigen_ground.row(j++) << p.x, p.y, p.z;
    }
    Eigen::MatrixX3f centered = eigen_ground.rowwise() - eigen_ground.colwise().mean();
    Eigen::MatrixX3f cov = (centered.adjoint() * centered) / double(eigen_ground.rows() - 1);
    
    pc_mean_.resize(3);
    pc_mean_ << eigen_ground.colwise().mean()(0), eigen_ground.colwise().mean()(1), eigen_ground.colwise().mean()(2);

    Eigen::JacobiSVD<Eigen::MatrixX3f> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();
       
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    
    if (normal_(2) < 0) { for(int i=0; i<3; i++) normal_(i) *= -1; }

    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

void PatchWorkpp::extract_initial_seeds(
        const int zone_idx, const vector<PointXYZ> &p_sorted,
        vector<PointXYZ> &init_seeds, double th_seed) {
    
    init_seeds.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.size(); i++) {
            if (p_sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
                ++init_idx;
            } else {
                break;
            }
        }
    }
    
    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.size() && cnt < params_.num_lpr; i++) {
        sum += p_sorted[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0


    int init_seeds_num = 0;    // iterate pointcloud, filter those height is less than lpr.height+params_.th_seeds
    for (int i = 0; i < p_sorted.size(); i++) {
        if (p_sorted[i].z < lpr_height + th_seed) {
            init_seeds.push_back(p_sorted[i]);
        }
    }
}

void PatchWorkpp::extract_initial_seeds(
        const int zone_idx, const vector<PointXYZ> &p_sorted,
        vector<PointXYZ> &init_seeds) {

    init_seeds.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.size(); i++) {
            if (p_sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
                ++init_idx;
            } else {
                break;
            }
        }
    }
    
    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.size() && cnt < params_.num_lpr; i++) {
        sum += p_sorted[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    int init_seeds_num = 0;
    // iterate pointcloud, filter those height is less than lpr.height+params_.th_seeds
    for (int i = 0; i < p_sorted.size(); i++) {
        if (p_sorted[i].z < lpr_height + params_.th_seeds) {
            init_seeds.push_back(p_sorted[i]);
        }
    }
}

void PatchWorkpp::estimateGround(Eigen::MatrixXf cloud_in) {
    
    cloud_ground_.clear();
    cloud_nonground_.clear();

    if (params_.verbose) cout << "\033[1;32m" << "PatchWorkpp::estimateGround() - Estimation starts !" << "\033[0m" << endl;

    clock_t beg = clock();
    
    // 1. Reflected Noise Removal (RNR)
    if (params_.enable_RNR) reflected_noise_removal(cloud_in);

    clock_t t1 = clock();

    // 2. Concentric Zone Model (CZM)
    flush_patches(ConcentricZoneModel_);

    clock_t t1_1 = clock();

    pc2czm(cloud_in, ConcentricZoneModel_);

    clock_t t2 = clock();

    int concentric_idx = 0;

    centers_.clear();
    normals_.clear();

    double t_flush = t1_1-t1, t_czm = t2-t1_1, t_sort = 0.0, t_pca = 0.0, t_gle = 0.0, t_revert = 0.0, t_update = 0.0;

    std::vector<patchwork::RevertCandidate> candidates;
    std::vector<double> ringwise_flatness;
    
    for (int zone_idx = 0; zone_idx < params_.num_zones; ++zone_idx) {
        
        auto zone = ConcentricZoneModel_[zone_idx];

        for (int ring_idx = 0; ring_idx < params_.num_rings_each_zone[zone_idx]; ++ring_idx) {
            for (int sector_idx = 0; sector_idx < params_.num_sectors_each_zone[zone_idx]; ++sector_idx) {
                
                if (zone[ring_idx][sector_idx].size() < params_.num_min_pts)
                {
                    addCloud(cloud_nonground_, zone[ring_idx][sector_idx]);
                    continue;
                }

                // --------- region-wise sorting (faster than global sorting method) ---------------- //
                clock_t t_bef_sort = clock();
                sort(zone[ring_idx][sector_idx].begin(), zone[ring_idx][sector_idx].end(), point_z_cmp);
                clock_t t_aft_sort = clock();

                t_sort += t_aft_sort - t_bef_sort;
                // ---------------------------------------------------------------------------------- //

                clock_t t_bef_pca = clock();
                extract_piecewiseground(zone_idx, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);
                clock_t t_aft_pca = clock();

                t_pca += t_aft_pca - t_bef_pca;

                centers_.push_back(PointXYZ(pc_mean_(0), pc_mean_(1), pc_mean_(2)));
                normals_.push_back(PointXYZ(normal_(0), normal_(1), normal_(2)));

                clock_t t_bef_gle = clock();
                // Status of each patch
                // used in checking uprightness, elevation, and flatness, respectively
                const double ground_uprightness = normal_(2);
                const double ground_elevation   = pc_mean_(2);
                const double ground_flatness    = singular_values_.minCoeff();
                const double line_variable      = singular_values_(1) != 0 ? singular_values_(0)/singular_values_(1) : std::numeric_limits<double>::max();
                
                double heading = 0.0;
                for(int i=0; i<3; i++) heading += pc_mean_(i)*normal_(i);

                /*  
                    About 'is_heading_outside' condition, heading should be smaller than 0 theoretically.
                    ( Imagine the geometric relationship between the surface normal vector on the ground plane and 
                        the vector connecting the sensor origin and the mean point of the ground plane )

                    However, when the patch is far awaw from the sensor origin, 
                    heading could be larger than 0 even if it's ground due to lack of amount of ground plane points.
                    
                    Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near condition )
                */
                bool is_upright         = ground_uprightness > params_.uprightness_thr;
                bool is_not_elevated    = ground_elevation < params_.elevation_thr[concentric_idx];
                bool is_flat            = ground_flatness < params_.flatness_thr[concentric_idx];
                bool is_near_zone       = concentric_idx < params_.num_rings_of_interest;
                bool is_heading_outside = heading < 0.0;

                /*
                    Store the elevation & flatness variables
                    for A-GLE (Adaptive Ground Likelihood Estimation)
                    and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
                */
                if (is_upright && is_not_elevated && is_near_zone)
                {
                    update_elevation_[concentric_idx].push_back(ground_elevation);
                    update_flatness_[concentric_idx].push_back(ground_flatness);

                    ringwise_flatness.push_back(ground_flatness);
                }

                // Ground estimation based on conditions
                if (!is_upright)
                {
                    addCloud(cloud_nonground_, regionwise_ground_);
                }
                else if (!is_near_zone)
                {
                    addCloud(cloud_ground_, regionwise_ground_);
                }
                else if (!is_heading_outside)
                {
                    addCloud(cloud_nonground_, regionwise_ground_);
                }
                else if (is_not_elevated || is_flat)
                {
                    addCloud(cloud_ground_, regionwise_ground_);
                }
                else
                {
                    patchwork::RevertCandidate candidate(concentric_idx, sector_idx, ground_flatness, line_variable, pc_mean_, regionwise_ground_);
                    candidates.push_back(candidate);
                }
                // Every regionwise_nonground is considered nonground.
                addCloud(cloud_nonground_, regionwise_nonground_);

                clock_t t_aft_gle = clock();

                t_gle += t_aft_gle - t_bef_gle;
            }

            clock_t t_bef_revert = clock();
            if (!candidates.empty())
            {
                if (params_.enable_TGR) {
                    temporal_ground_revert(ringwise_flatness, candidates, concentric_idx);
                } else {
                    for ( auto candidate : candidates ) {
                        addCloud(cloud_nonground_, candidate.regionwise_ground);
                    }
                }

                candidates.clear();
                ringwise_flatness.clear();
            }
            clock_t t_aft_revert = clock();
            
            t_revert += t_aft_revert - t_bef_revert;

            concentric_idx++;
        }
    }

    clock_t t_bef_update = clock();
    update_elevation_thr();
    update_flatness_thr();
    clock_t t_aft_update = clock();

    t_update = t_aft_update - t_bef_update;

    clock_t end = clock();
    time_taken_ = end - beg;

    if (params_.verbose)
    {
        cout << "Time taken : " << time_taken_ / double(1000000) << "(sec) ~ "
            //  << t_flush / double(1000000)  << "(flush) + " 
             << t_czm / double(1000000)  << "(czm) + " 
             << t_sort / double(1000000)  << "(sort) + " 
             << t_pca / double(1000000) << "(pca) + "
             << t_gle / double(1000000) << "(estimate)" << endl;
            //  << t_revert / double(1000000) << "(revert) + "
            //  << t_update / double(1000000) << "(update)" << endl;
    }

    if (params_.verbose) cout << "\033[1;32m" << "PatchWorkpp::estimateGround() - Estimation is finished !" << "\033[0m" << endl;
}

void PatchWorkpp::update_elevation_thr(void)
{
    for (int i=0; i<params_.num_rings_of_interest; i++)
    {
        if (update_elevation_[i].empty()) continue;

        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_elevation_[i], update_mean, update_stdev);
        if (i==0) {
            params_.elevation_thr[i] = update_mean + 3*update_stdev;
            params_.sensor_height = -update_mean;
        }
        else params_.elevation_thr[i] = update_mean + 2*update_stdev;

        // if (params_.verbose) cout << "elevation threshold [" << i << "]: " << params_.elevation_thr[i] << endl;

        int exceed_num = update_elevation_[i].size() - params_.max_elevation_storage;
        if (exceed_num > 0) update_elevation_[i].erase(update_elevation_[i].begin(), update_elevation_[i].begin() + exceed_num);
    }
}

void PatchWorkpp::update_flatness_thr(void)
{
    for (int i=0; i<params_.num_rings_of_interest; i++)
    {
        if (update_flatness_[i].empty()) break;
        if (update_flatness_[i].size() <= 1) break;

        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_flatness_[i], update_mean, update_stdev);
        params_.flatness_thr[i] = update_mean+update_stdev;

        // if (params_.verbose) { cout << "flatness threshold [" << i << "]: " << params_.flatness_thr[i] << endl; }

        int exceed_num = update_flatness_[i].size() - params_.max_flatness_storage;
        if (exceed_num > 0) update_flatness_[i].erase(update_flatness_[i].begin(), update_flatness_[i].begin() + exceed_num);
    }
}

void PatchWorkpp::reflected_noise_removal(Eigen::MatrixXf &cloud_in)
{
    if ( cloud_in.cols() < 4 ) {
        cout << "RNR requires intensity information !" << endl;
        return;
    }

    int cnt = 0;
    for (int i=0; i<cloud_in.rows(); i++)
    {
        double r = sqrt( cloud_in.row(i)(0)*cloud_in.row(i)(0) + cloud_in.row(i)(1)*cloud_in.row(i)(1) );
        double z = cloud_in.row(i)(2);
        double ver_angle_in_deg = atan2(z, r)*180/M_PI;

        if ( ver_angle_in_deg < params_.RNR_ver_angle_thr && z < -params_.sensor_height-0.8 && cloud_in.row(i)(3) < params_.RNR_intensity_thr)
        {
            cloud_nonground_.push_back(PointXYZ(cloud_in.row(i)(0), cloud_in.row(i)(1), cloud_in.row(i)(2), cloud_in.row(i)(3)));
            cloud_in.row(i)(2) = std::numeric_limits<float>::min();
            cnt++;
        }
    }

    if ( params_.verbose ) cout << "PatchWorkpp::reflected_noise_removal() - Number of Noises : " << cnt << endl;
}

void PatchWorkpp::temporal_ground_revert(std::vector<double> ring_flatness, std::vector<patchwork::RevertCandidate> candidates,
                                         int concentric_idx)
{
    if (params_.verbose) std::cout << "\033[1;34m" << "=========== Temporal Ground Revert (TGR) ===========" << "\033[0m" << endl;

    double mean_flatness = 0.0, stdev_flatness = 0.0;
    calc_mean_stdev(ring_flatness, mean_flatness, stdev_flatness);
    
    if (params_.verbose)
    {
        cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
             << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness << std::endl;
    }

    for( auto candidate : candidates )
    {        
        // Debug
        if(params_.verbose)
        {
            cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
                 << " / flatness: " << candidate.ground_flatness
                 << " / line_variable: " << candidate.line_variable
                 << " / ground_num : " << candidate.regionwise_ground.size() 
                 << "\033[0m" << endl;
        }

        double mu_flatness = mean_flatness + 1.5*stdev_flatness;
        double prob_flatness = 1/(1+exp( (candidate.ground_flatness-mu_flatness)/(mu_flatness/10) ));

        if (candidate.regionwise_ground.size() > 1500 && candidate.ground_flatness < params_.th_dist*params_.th_dist) prob_flatness = 1.0;
       
        double prob_line = 1.0;
        if (candidate.line_variable > 8.0 )//&& candidate.line_dir > M_PI/4)//
        {
            // if (params_.verbose) cout << "line_dir: " << candidate.line_dir << endl;
            prob_line = 0.0;
        }

        bool revert = prob_line*prob_flatness > 0.5;

        if ( concentric_idx < params_.num_rings_of_interest )
        {
            if (revert)
            {
                if (params_.verbose)
                {
                    cout << "\033[1;32m" << "REVERT TRUE" << "\033[0m" << endl;
                }
                addCloud(cloud_ground_, candidate.regionwise_ground);
            }
            else
            {
                if (params_.verbose) 
                {
                    cout << "\033[1;31m" << "FINAL REJECT" << "\033[0m" << endl;
                }
                addCloud(cloud_nonground_, candidate.regionwise_ground);
            }                
        }
    }

    if (params_.verbose) std::cout << "\033[1;34m" << "====================================================" << "\033[0m" << endl;
}

// For adaptive
void PatchWorkpp::extract_piecewiseground(
        const int zone_idx, const vector<PointXYZ> &src,
        vector<PointXYZ> &dst,
        vector<PointXYZ> &non_ground_dst) {
    
    // 0. Initialization
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();

    // 1. Region-wise Vertical Plane Fitting (R-VPF) 
    // : removes potential vertical plane under the ground plane
    vector<PointXYZ> src_wo_verticals;
    src_wo_verticals = src;
    
    if (params_.enable_RVPF)
    {
        for (int i = 0; i < params_.num_iter; i++)
        {
            extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, params_.th_seeds_v);
            estimate_plane(ground_pc_);

            if (zone_idx == 0 && normal_(2) < params_.uprightness_thr)
            {
                vector<PointXYZ> src_tmp;
                src_tmp = src_wo_verticals;
                src_wo_verticals.clear();

                for ( auto point : src_tmp ) {

                    double distance = calc_point_to_plane_d(point, normal_, d_);
                    
                    if ( abs(distance) < params_.th_dist_v ) {
                        non_ground_dst.push_back(point);
                    } else {
                        src_wo_verticals.push_back(point);
                    }
                }
            }
            else break;
        }
    }

    // 2. Region-wise Ground Plane Fitting (R-GPF)
    // : fits the ground plane

    extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
    estimate_plane(ground_pc_);

    for (int i = 0; i < params_.num_iter; i++) {

        ground_pc_.clear();

        for ( auto point : src_wo_verticals ) {
            
            double distance = calc_point_to_plane_d(point, normal_, d_);
            
            if (i < params_.num_iter - 1) {
                if ( distance < params_.th_dist ) {
                    ground_pc_.push_back(point);
                }
            } else {
                if ( distance < params_.th_dist ) {
                    dst.push_back(point);
                } else {
                    non_ground_dst.push_back(point);
                }
            }
        }

        if (i < params_.num_iter -1) {
            estimate_plane(ground_pc_);
        }
        else {
            estimate_plane(dst);
        }
    }

    if ( dst.size() + non_ground_dst.size() != src.size() ) {
        cout << "\033[1;33m" << "Points are Missing/Adding !!! Please Check !! " << "\033[0m" << endl;
        cout << "gnd size: " << dst.size() << ", non gnd size: " << non_ground_dst.size() << ", src: " << src.size() << endl;
    }
}

double PatchWorkpp::calc_point_to_plane_d(PointXYZ p, Eigen::VectorXf normal, double d)
{
    return normal(0)*p.x+normal(1)*p.y+normal(2)*p.z+d;
}


void PatchWorkpp::calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev)
{
    if (vec.size() <= 1) return;

    mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

    for (int i=0; i<vec.size(); i++) { stdev += (vec.at(i)-mean)*(vec.at(i)-mean); }  
    stdev /= vec.size()-1;
    stdev = sqrt(stdev);
}

double PatchWorkpp::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    double angle = atan2(y, x);
    return angle > 0 ? angle : 2*M_PI+angle;
}

double PatchWorkpp::xy2radius(const double &x, const double &y) {
    // return sqrt(pow(x, 2) + pow(y, 2));
    return sqrt(x*x + y*y);
}

void PatchWorkpp::pc2czm(const Eigen::MatrixXf &src, std::vector<Zone> &czm) {

    double max_range = params_.max_range, min_range = params_.min_range;
    double min_range_0 = min_ranges_[0], min_range_1 = min_ranges_[1], min_range_2 = min_ranges_[2], min_range_3 = min_ranges_[3];
    int num_ring_0 = params_.num_rings_each_zone[0], num_sector_0 = params_.num_sectors_each_zone[0];
    int num_ring_1 = params_.num_rings_each_zone[1], num_sector_1 = params_.num_sectors_each_zone[1];
    int num_ring_2 = params_.num_rings_each_zone[2], num_sector_2 = params_.num_sectors_each_zone[2];
    int num_ring_3 = params_.num_rings_each_zone[3], num_sector_3 = params_.num_sectors_each_zone[3];

    for (int i=0; i<src.rows(); i++) {

        float x = src.row(i)(0), y = src.row(i)(1), z = src.row(i)(2), intensity = src.row(i)(3);

        if ( z == std::numeric_limits<float>::min() ) continue;

        double r = xy2radius(x, y);
        int ring_idx, sector_idx;
        if ((r <= max_range) && (r > min_range)) {
            // double theta = xy2theta(pt.x, pt.y);
            double theta = xy2theta(x, y);
            
            if (r < min_range_1) { // In First rings
                ring_idx = min(static_cast<int>(((r - min_range_0) / ring_sizes_[0])), num_ring_0 - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[0])), num_sector_0 - 1);
                czm[0][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, intensity));
            } else if (r < min_range_2) {
                ring_idx = min(static_cast<int>(((r - min_range_1) / ring_sizes_[1])), num_ring_1 - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[1])), num_sector_1 - 1);
                czm[1][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, intensity));
            } else if (r < min_range_3) {
                ring_idx = min(static_cast<int>(((r - min_range_2) / ring_sizes_[2])), num_ring_2 - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[2])), num_sector_2 - 1);
                czm[2][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, intensity));
            } else { // Far!
                ring_idx = min(static_cast<int>(((r - min_range_3) / ring_sizes_[3])), num_ring_3 - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[3])), num_sector_3 - 1);
                czm[3][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, intensity));
            }

        } else {
            cloud_nonground_.push_back(PointXYZ(x, y, z, intensity));
        }
    }
    if (params_.verbose) cout << "\033[1;33m" << "PatchWorkpp::pc2czm() - Divides pointcloud into the concentric zone model successfully" << "\033[0m" << endl;
}

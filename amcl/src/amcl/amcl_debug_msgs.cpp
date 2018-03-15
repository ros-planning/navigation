#include "amcl_debug_msgs.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace amcl;
typedef Eigen::Matrix<double, 6, 6> Covariance3D;

void AmclDebug::publish_pose(const geometry_msgs::PoseWithCovarianceStamped& p) const
{
    //printf("Published pose stamped %lu\n", p.header.stamp.toNSec());
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose = p.pose.pose;
    pose_msg.header = p.header;
    pose_pub_.publish(pose_msg);

    publish_cov(p.pose, &pose_msg.header);
}

void AmclDebug::publish_cov(const geometry_msgs::PoseWithCovariance& p, std_msgs::Header* h) const
{
    visualization_msgs::Marker cov_msg;
    if (h)
    {
        cov_msg.header = *h;
    }
    else
    {
        cov_msg.header.stamp = ros::Time::now();
        cov_msg.header.frame_id = global_frame_id;
    }
    cov_msg.ns = "pose_cov";

    cov_msg.type = visualization_msgs::Marker::SPHERE;
    pf_sample_set_t* current_set = curr_set();
    cov_msg.color.r = current_set->converged? 0.1 : 1.0;
    cov_msg.color.b = 1.0;
    cov_msg.color.g = current_set->converged? 1.0 : 0.1;
    cov_msg.color.a = 0.5;
    cov_msg.pose.position = p.pose.position;
    cov_msg.lifetime = lifetime;

    // Compute eigen values
    Covariance3D cov;
    double* arr_cov = cov.data();
    for (size_t idx = 0; idx < 36; ++idx) // 36: 6x6 covariance matrix
    {
        arr_cov[idx] = p.covariance[idx];
    }

    if (fillCovMsg(cov, cov_msg)) marker_pub_.publish(cov_msg);
    else
    {
        std::ostringstream err("");
        err << "Eigenvalues too small for pose @ t = " << cov_msg.header.stamp.toNSec() << " - position " << p.pose.position.x << ", " << p.pose.position.y << std::endl;
        err << "Cov nan? \n" << cov << std::endl;
        std::cerr << err.str() << std::endl;
    }
    std::cout << "Converged? score = " << current_set->score << " -> " << std::boolalpha << current_set->converged << std::endl;
}

bool AmclDebug::fillCovMsg(const Covariance3D& cov, visualization_msgs::Marker& msg) const
{
    double lamda1 = 0;
    double lamda2 = 0;
    size_t lamdx1, lamdx2;
    for (size_t lamdx = 0; lamdx < 6; ++lamdx)
    {
        double lamda_temp = cov.eigenvalues()[lamdx].real();
        if (lamda_temp > lamda1)
        {
            lamda1 = lamda_temp;
            lamdx1  = lamdx;
        }
        else if (lamda_temp > lamda2)
        {
            lamda2 = lamda_temp;
            lamdx2 = lamdx;
        }
    }

    if (lamda1 == 0 || lamda2 == 0)
    {
        if ( lamda1 * lamda2 == 0 ) return false;
        else
        {
            if (lamda1 == 0) lamda1 = 1e-24;
            if (lamda2 == 0) lamda2 = 1e-24;
        }
    }

    // Calculate ellipse orientation
    double angle;
    {
        Eigen::EigenSolver<Covariance3D> es(cov);
        Eigen::VectorXcd ev = es.eigenvectors().col(lamdx1);
        angle = atan2(ev(1).real(), ev(0).real());
    }
    msg.pose.orientation.z = sin(angle/2.0);
    msg.pose.orientation.w = cos(angle/2.0);

    // Calculate ellipse axes
    double half_major_axis = 2.0 * sqrt(5.991 * lamda1); // 2.4477: chisquare_val for 95% confidence interval
    double half_minor_axis = 2.0 * sqrt(5.991 * lamda2);
    msg.scale.x = half_major_axis;
    msg.scale.y = half_minor_axis;
    msg.scale.z = 0.1; // assuming no z

    return true;
}

void AmclDebug::publish_scan_at_pose(sensor_msgs::LaserScan laser_scan, std::vector<double> intensities)
{
    if (! intensities.size() || ! laser_scan.ranges.size()) return;

    decltype(laser_scan.ranges) sub_sampled_ranges;
    decltype(laser_scan.intensities) sub_sampled_intensties;

    for (size_t idx = 0; idx < intensities.size(); ++idx)
    {
        if (!std::isnan(intensities[idx]))
        {
            if (laser_scan.intensities[idx] == 0.0 && intensities[idx] >= 0.99) sub_sampled_intensties.push_back(0.5*244); //  points out of map
            else sub_sampled_intensties.push_back(intensities[idx] * laser_scan.intensities[idx]);
            sub_sampled_ranges.push_back(laser_scan.ranges[idx]);
        }
    }

    if (! sub_sampled_ranges.size())
    {
        ROS_WARN("Empty downsampled intensities");
        return;
    }
    if (sub_sampled_ranges.size() != sub_sampled_intensties.size())
    {
        ROS_WARN("Vectors of sub-sampled ranges and intensities have different length! ranges.size() = %lu vs intensities.size() = %lu",
            sub_sampled_ranges.size(),
            sub_sampled_intensties.size());;
        return;
    }

    size_t pre_size = laser_scan.ranges.size();
    laser_scan.angle_increment *= ((float)pre_size - 1.0)
        /((float)sub_sampled_ranges.size() - 1.0); // as step calculation in AMCLLaser::LikelihoodFieldModel
    laser_scan.ranges = sub_sampled_ranges;
    laser_scan.intensities = sub_sampled_intensties;
    // laser_scan.angle_min unchanged
    laser_scan.angle_max = laser_scan.angle_min
        + laser_scan.angle_increment * (float)sub_sampled_ranges.size();
    pose_scan_pub_.publish(laser_scan);
}

void AmclDebug::publish_cluster( size_t id, const double& weight, pf_vector_t *mean, pf_matrix_t *cov) const
{
    pf_sample_set_t* current_set = curr_set();
    if (current_set->cluster_count > 20 || ! weight) return;

    visualization_msgs::Marker cluster_msg;
    cluster_msg.header.stamp = ros::Time::now();
    cluster_msg.header.frame_id = global_frame_id;
    cluster_msg.ns = "clusters";
    cluster_msg.id = id;
    cluster_msg.type = visualization_msgs::Marker::ARROW;

    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(mean->v[2]),
                            tf::Vector3(mean->v[0], mean->v[1], 0)),
                            cluster_msg.pose);

    cluster_msg.scale.x = 1.0;// + sqrt(cov->m[0][0]);
    cluster_msg.scale.y = 0.1;// + sqrt(cov->m[1][1]);
    cluster_msg.scale.z = 0.1;// + sqrt(cov->m[2][2]);
    cluster_msg.color.r = 1.0 - weight;
    cluster_msg.color.g = weight;
    cluster_msg.color.b = current_set->score;
    cluster_msg.color.a = current_set->converged? 1.0 : 0.5;
    cluster_msg.lifetime = lifetime;
    marker_pub_.publish(cluster_msg);

    visualization_msgs::Marker text_msg;
    text_msg.header = cluster_msg.header;
    text_msg.ns = "cluster_id";
    text_msg.id = id;
    text_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    text_msg.text = "";

    bool only_cluster = current_set->clusters[id].count == current_set->sample_count;
    if (! only_cluster)
    {
        text_msg.text = std::to_string(id+1) + "/" + std::to_string(current_set->cluster_count) + "; w " + std::to_string(weight);
    }
    size_t p_cluster = current_set->clusters[id].count;
    size_t p_total = current_set->sample_count;
    text_msg.text += " #p " + std::to_string(p_cluster);

    if (! only_cluster) text_msg.text += "/" + std::to_string(p_total);
    else text_msg.text += " / score " + std::to_string(current_set->score);

    text_msg.pose.position.x = cluster_msg.pose.position.x + 0.5*(float)(id+1);
    text_msg.pose.position.y = cluster_msg.pose.position.y + 0.5*(float)(id+1);
    text_msg.pose.position.z = cluster_msg.pose.position.z + 0.1;
    text_msg.scale.z = 0.5;//0.6 + 0.5*(only_cluster? 0.0 : (float)(p_total - (*pf)->min_samples) /(float)( (*pf)->max_samples - (*pf)->min_samples));
    text_msg.color.r = cluster_msg.color.r;//std::max(cluster_msg.color.r * 2.0, 1.0);
    text_msg.color.g = cluster_msg.color.g;//std::max(cluster_msg.color.g * 2.0, 1.0);
    text_msg.color.b = current_set->score;
    text_msg.color.a = 1.0;
    text_msg.lifetime = cluster_msg.lifetime;
    marker_pub_.publish(text_msg);

    if (current_set->clusters[id].weight > 0.0) publish_cov(cluster_msg, cov, ! id); // FIXME scale = 0 error in rviz??
}
void AmclDebug::publish_cov(const visualization_msgs::Marker& cluster_msg, pf_matrix_t *cov_ptr, bool clear) const
{
    visualization_msgs::Marker cov_msg;
    cov_msg.header = cluster_msg.header;
    cov_msg.ns = "cluster_cov";
    cov_msg.id = cluster_msg.id;
    cov_msg.type = visualization_msgs::Marker::SPHERE;
    cov_msg.color = cluster_msg.color;
    cov_msg.color.a = 0.3;
    cov_msg.pose.position = cluster_msg.pose.position;
    cov_msg.lifetime = cluster_msg.lifetime;

    // Compute eigen values
    Covariance3D cov;
    cov << cov_ptr->m[0][0], cov_ptr->m[0][1], 0,0,0, cov_ptr->m[0][2],
            cov_ptr->m[1][0], cov_ptr->m[1][1], 0,0,0, cov_ptr->m[1][2],
            0,0,0,0,0,0,
            0,0,0,0,0,0,
            0,0,0,0,0,0,
            cov_ptr->m[2][0], cov_ptr->m[2][1], 0,0,0, cov_ptr->m[2][2];

    if (fillCovMsg(cov, cov_msg)) marker_pub_.publish(cov_msg);
}

void AmclDebug::publish_clusters_cloud(pf_sample_set_t* set) const
{
    if (! set) set = curr_set();
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id;
    double weight_thresh = 1.01/set->sample_count;
    size_t count = 0;
    for(int i=0;i<set->sample_count;i++)
    {
      count += (set->samples[i].weight > weight_thresh);
    }
    cloud_msg.poses.resize(count);

    if (! count)
    {
        heavy_particles_cloud_pub_.publish(cloud_msg);
        return;
    }

    std::ostringstream msg("");
    msg << count << " poses (out of " << set->sample_count << ") weight > " << weight_thresh;
    std::string msg_str = msg.str();
    ROS_INFO("%s", msg_str.c_str());

    count = 0;
    for(int i=0;i < set->sample_count;++i)
    {
      if (set->samples[i].weight <= weight_thresh) continue;
      pf_sample_t *sample = set->samples  + i;
      //std::cout << "\t sample " << i << " weighs " << sample->weight << " (" << sample->pose.v[0] << ", " << sample->pose.v[1] << " yaw " << sample->pose.v[2] << std::endl;
      tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample->pose.v[2]),
                                tf::Vector3(sample->pose.v[0],
                                         sample->pose.v[1], 0)),
                                cloud_msg.poses[count++]);
    }

    heavy_particles_cloud_pub_.publish(cloud_msg);
}

void AmclDebug::outputMap(map_t* map, std::string name)
{
    cv::Mat output = 127*cv::Mat::ones(cv::Size(map->size_x, map->size_y), CV_8UC1);
    for(int i=0; i < map->size_x; ++i)
    {
        for(int j=0; j < map->size_y; ++j)
        {
            int occ_state = map->cells[MAP_INDEX(map, i, j)].occ_state;
            if (occ_state == 1) output.at<uchar>(j, i) = 255;
            else if (occ_state == -1) output.at<uchar>(j, i) = 0;
        }
    }
    cv::imwrite(name, output);
    std::cout << "Wrote to image " << name << std::endl;
}

void AmclDebug::outputLikelihoodField(map_t* map, std::string name)
{
    cv::Mat output = cv::Mat::zeros(cv::Size(map->size_x, map->size_y), CV_8UC1);
    for(int i=0; i < map->size_x; ++i)
    {
        for(int j=0; j < map->size_y; ++j)
        {
            float dist = map->cells[MAP_INDEX(map, i, j)].occ_dist;
            size_t val = (1.0 - dist/map->max_occ_dist)*255.0;
            output.at<uchar>(j, i) = val;
        }
    }
    cv::imwrite(name, output);
    std::cout << "Wrote to image " << name << std::endl;
}

#define OCC_PERC(map, cell_i) exp( - pow(map->cells[cell_i].occ_dist, 2) / pow(map->max_occ_dist * 0.34, 2))

void AmclDebug::publish_likelihood_field(map_t* map)
{
    nav_msgs::OccupancyGrid field_msg;
    field_msg.header.stamp = ros::Time::now();
    field_msg.header.frame_id = global_frame_id;
    field_msg.header.seq = seq.fetch_add(1);
    field_msg.info.width = map->size_x;
    field_msg.info.height = map->size_y;
    field_msg.info.origin.position.x = MAP_WXGX(map, 0);
    field_msg.info.origin.position.y = MAP_WYGY(map, 0);
    field_msg.info.resolution = map->scale;
    size_t map_size = map->size_x * map->size_y;
    std::vector<int8_t> map_data(map_size);

    for(int i=0;i<map_size; ++i)
    {
        map_data[i] = OCC_PERC(map, i) * 244.0;
    }

    field_msg.data = map_data;

    likelihood_field_pub_.publish(field_msg);

    ROS_INFO("Published likelihood field with dimensions %d X %d @ %.3f m/pix oriented %.3f\n",
             field_msg.info.width,
             field_msg.info.height,
             field_msg.info.resolution,
             0.0);
}

#include "lvio_fusion/visual/local_map.h"
#include "lvio_fusion/map.h"
#include "lvio_fusion/utility.h"
#include "lvio_fusion/visual/camera.h"

namespace lvio_fusion
{

inline cv::Mat brief2mat(BRIEF &brief)
{
    return cv::Mat(1, 32, CV_8U, reinterpret_cast<uchar *>(&brief));
}

inline BRIEF mat2brief(const cv::Mat &mat)
{
    BRIEF brief;
    memcpy(&brief, mat.data, 32);
    return brief;
}

inline std::vector<BRIEF> mat2briefs(cv::Mat descriptors)
{
    std::vector<BRIEF> briefs;
    for (int i = 0; i < descriptors.rows; i++)
    {
        briefs.push_back(mat2brief(descriptors.row(i)));
    }
    return briefs;
}

inline cv::Mat briefs2mat(std::vector<BRIEF> briefs)
{
    cv::Mat descriptors(briefs.size(), 32, CV_8U);
    for (int i = 0; i < briefs.size(); i++)
    {
        brief2mat(briefs[i]).copyTo(descriptors.row(i));
    }
    return descriptors;
}

inline Vector3d LocalMap::ToWorld(Point::Ptr feature)
{
    Vector3d pb = Camera::Get(1)->Pixel2Robot(
        cv2eigen(feature->landmark->first_observation->keypoint),
        feature->landmark->depth);
    return Camera::Get()->Robot2World(pb, pose_cache[feature->frame->time]);
}

int LocalMap::Init(Frame::Ptr new_kf)
{
    // get feature pyramid
    local_features_[new_kf->time] = Pyramid();
    GetFeaturePyramid(new_kf, local_features_[new_kf->time]);
    GetNewLandmarks(new_kf, local_features_[new_kf->time]);
    return GetLandmarks(new_kf).size();
}

void LocalMap::Reset()
{
    local_features_.clear();
}

void LocalMap::AddKeyFrame(Frame::Ptr new_kf)
{
    // get feature pyramid
    local_features_[new_kf->time] = Pyramid();
    GetFeaturePyramid(new_kf, local_features_[new_kf->time]);
    // search
    pose_cache[new_kf->time] = new_kf->pose;
    std::vector<double> kfs = GetCovisibilityKeyFrames(new_kf);
    GetNewLandmarks(new_kf, local_features_[new_kf->time]);
    Search(kfs, new_kf);
    InsertNewLandmarks(new_kf);
    // remove old key frames
    if (local_features_.size() > windows_size_)
    {
        local_features_.erase(local_features_.begin());
        oldest = local_features_.begin()->first;
    }
}

void LocalMap::UpdateCache()
{
    std::unique_lock<std::mutex> lock(mutex_);
    map_.clear();
    pose_cache.clear();
    position_cache.clear();
    for (auto &pair : local_features_)
    {
        pose_cache[pair.first] = Map::Instance().GetKeyFrame(pair.first)->pose;
        for (auto &features : pair.second)
        {
            for (auto &feature : features)
            {
                if (feature->landmark && position_cache.find(feature->landmark->id) == position_cache.end())
                {
                    position_cache[feature->landmark->id] = std::make_pair(feature->landmark->depth, ToWorld(feature));
                    if (!feature->match)
                    {
                        map_[feature->landmark->id] = feature;
                    }
                }
            }
        }
    }
}

void LocalMap::InsertNewLandmarks(Frame::Ptr frame)
{
    for (auto &pair : frame->features_left)
    {
        auto landmark = pair.second->landmark.lock();
        if (Map::Instance().landmarks.find(landmark->id) == Map::Instance().landmarks.end())
        {
            auto iter = map_.find(landmark->id);
            if (iter != map_.end() && !iter->second->match && !iter->second->insert)
            {
                iter->second->insert = true;
            }
            auto first_frame = landmark->FirstFrame().lock();
            first_frame->AddFeature(landmark->first_observation);
            first_frame->AddFeature(landmark->observations.begin()->second);
            Map::Instance().InsertLandmark(landmark);
        }
    }
}

cv::Mat img_track;
void LocalMap::GetFeaturePyramid(Frame::Ptr frame, Pyramid &pyramid)
{
    cv::cvtColor(frame->image_left, img_track, cv::COLOR_GRAY2RGB);
    cv::Mat mask = cv::Mat(frame->image_left.size(), CV_8UC1, 255);
    for (auto &pair_feature : frame->features_left)
    {
        cv::circle(mask, pair_feature.second->keypoint, 20, 0, cv::FILLED);
    }
    std::vector<cv::KeyPoint> kps;
    cv::Mat descriptors;
    extractor_.DetectAndCompute(frame->image_left, mask, kps, descriptors);
    // int part_width = frame->image_left.cols / 2, part_height = frame->image_left.rows / 2;
    // cv::Rect parts[4] = {cv::Rect(0, 0, part_width, part_height),
    //                      cv::Rect(part_width, 0, part_width, part_height),
    //                      cv::Rect(0, part_height, part_width, part_height),
    //                      cv::Rect(part_width, part_height, part_width, part_height)};
    // for (int i = 0; i < 4; i++)
    // {
    //     std::vector<cv::KeyPoint> part_kps;
    //     cv::Mat part_descriptors;
    //     orb_detect_and_compute(frame->image_left(parts[i]), mask(parts[i]), part_kps, part_descriptors);
    //     kps.reserve(kps.size() + part_kps.size());
    //     for (auto &kp : part_kps)
    //     {
    //         kp.pt.x += parts[i].x;
    //         kp.pt.y += parts[i].y;
    //         kps.push_back(kp);
    //         cv::circle(img_track, kp.pt, 2, cv::Scalar(0, 255, 0), cv::FILLED);
    //     }
    //     if (descriptors.empty())
    //     {
    //         descriptors = part_descriptors;
    //     }
    //     else
    //     {
    //         cv::vconcat(descriptors, part_descriptors, descriptors);
    //     }
    // }

    std::vector<std::vector<int>> index_levels;
    index_levels.resize(num_levels_);
    for (int i = 0; i < kps.size(); i++)
    {
        index_levels[kps[i].octave].push_back(i);
    }
    pyramid.clear();
    for (int i = 0; i < num_levels_; i++)
    {
        Points level_features;
        for (int index : index_levels[i])
        {
            level_features.push_back(Point::Ptr(new Point(frame, kps[index], mat2brief(descriptors.row(index)))));
        }
        pyramid.push_back(level_features);
    }
    cv::imshow("d", img_track);
    cv::waitKey(1);
}

void LocalMap::GetNewLandmarks(Frame::Ptr frame, Pyramid &pyramid)
{
    Points featrues;
    for (int i = 0; i < num_levels_; i++)
    {
        for (auto &feature : pyramid[i])
        {
            if (!feature->landmark)
            {
                featrues.push_back(feature);
            }
        }
    }
    Triangulate(frame, featrues);
}

void LocalMap::Triangulate(Frame::Ptr frame, Points &featrues)
{
    std::vector<cv::Point2f> kps_left, kps_right;
    kps_left.resize(featrues.size());
    for (int i = 0; i < featrues.size(); i++)
    {
        kps_left[i] = featrues[i]->kp.pt;
    }
    kps_right = kps_left;
    std::vector<uchar> status;
    optical_flow(frame->image_left, frame->image_right, kps_left, kps_right, status);
    // triangulate new points
    for (int i = 0; i < kps_left.size(); ++i)
    {
        if (status[i])
        {
            // triangulation
            Vector2d kp_left = cv2eigen(kps_left[i]);
            Vector2d kp_right = cv2eigen(kps_right[i]);
            Vector3d pb = Vector3d::Zero();
            triangulate(Camera::Get()->extrinsic.inverse(), Camera::Get(1)->extrinsic.inverse(), Camera::Get()->Pixel2Sensor(kp_left), Camera::Get(1)->Pixel2Sensor(kp_right), pb);
            if ((Camera::Get()->Robot2Pixel(pb) - kp_left).norm() < 0.5 && (Camera::Get(1)->Robot2Pixel(pb) - kp_right).norm() < 0.5)
            {
                auto new_landmark = visual::Landmark::Create(Camera::Get(1)->Robot2Sensor(pb).z());
                auto new_left_feature = visual::Feature::Create(frame, kps_left[i], new_landmark);
                auto new_right_feature = visual::Feature::Create(frame, kps_right[i], new_landmark);
                new_right_feature->is_on_left_image = false;

                new_landmark->AddObservation(new_left_feature);
                new_landmark->AddObservation(new_right_feature);
                featrues[i]->landmark = new_landmark;
                position_cache[new_landmark->id] = std::make_pair(new_landmark->depth, ToWorld(featrues[i]));
                map_[new_landmark->id] = featrues[i];
            }
        }
    }
}

std::vector<double> LocalMap::GetCovisibilityKeyFrames(Frame::Ptr frame)
{
    std::vector<double> kfs;
    for (auto &pair : local_features_)
    {
        Vector3d last_heading = pose_cache[pair.first].so3() * Vector3d::UnitX();
        Vector3d heading = frame->pose.so3() * Vector3d::UnitX();
        double degree = vectors_degree_angle(last_heading, heading);
        if (degree < 30)
        {
            kfs.push_back(pair.first);
        }
    }
    kfs.pop_back();
    return kfs;
}

void LocalMap::Search(std::vector<double> kfs, Frame::Ptr frame)
{
    for (int i = 0; i < kfs.size(); i++)
    {
        Search(local_features_[kfs[i]], pose_cache[kfs[i]], local_features_[frame->time], frame);
    }
}

void LocalMap::Search(Pyramid &last_pyramid, SE3d last_pose, Pyramid &current_pyramid, Frame::Ptr frame)
{
    for (auto &features : current_pyramid)
    {
        for (auto &feature : features)
        {
            if (feature->landmark && !feature->match)
            {
                Search(last_pyramid, last_pose, feature, frame);
            }
        }
    }
}

void LocalMap::Search(Pyramid &last_pyramid, SE3d last_pose, Point::Ptr feature, Frame::Ptr frame)
{
    auto pc = Camera::Get()->World2Sensor(position_cache[feature->landmark->id].second, last_pose);
    if (pc.z() < 0)
        return;
    cv::Point2f p_in_last_left = eigen2cv(Camera::Get()->Sensor2Pixel(pc));
    Points features_in_radius;
    std::vector<BRIEF> briefs;
    int min_level = feature->kp.octave, max_level = feature->kp.octave + 1;
    for (int i = min_level; i <= max_level && i < num_levels_; i++)
    {
        double radius = 20 * scale_factors_[i];
        for (auto &last_feature : last_pyramid[i])
        {
            if (last_feature->landmark &&
                cv_distance(p_in_last_left, last_feature->kp.pt) < radius)
            {
                features_in_radius.push_back(last_feature);
                briefs.push_back(last_feature->brief);
            }
        }
    }

    cv::Mat descriptors_last = briefs2mat(briefs), descriptors_current = brief2mat(feature->brief);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(descriptors_current, descriptors_last, knn_matches, 2);
    const float ratio_thresh = 0.6;
    if (!features_in_radius.empty() && !knn_matches.empty() && knn_matches[0].size() == 2 &&
        knn_matches[0][0].distance < ratio_thresh * knn_matches[0][1].distance)
    {
        auto last_feature = features_in_radius[knn_matches[0][0].trainIdx];
        feature->landmark = last_feature->landmark;
        feature->match = true;

        auto new_left_feature = visual::Feature::Create(frame, feature->kp.pt, feature->landmark);
        feature->landmark->AddObservation(new_left_feature);
        frame->AddFeature(new_left_feature);
    }
}

LocalMap::Points LocalMap::GetLandmarks(Frame::Ptr frame)
{
    Points result;
    for (auto &features : local_features_[frame->time])
    {
        for (auto &feature : features)
        {
            if (feature->landmark)
            {
                result.push_back(feature);
            }
        }
    }
    return result;
}

PointRGBCloud LocalMap::GetLocalLandmarks()
{
    std::unique_lock<std::mutex> lock(mutex_);
    PointRGBCloud out;
    for (auto &pyramid : local_features_)
    {
        for (auto &features : pyramid.second)
        {
            for (auto &feature : features)
            {
                if (feature->insert)
                {
                    PointRGB point_color;
                    Vector3d pw = position_cache[feature->landmark->id].second;
                    point_color.x = pw.x();
                    point_color.y = pw.y();
                    point_color.z = pw.z();
                    point_color.r = 255;
                    point_color.g = 255;
                    point_color.b = 255;
                    out.push_back(point_color);
                }
            }
        }
    }
    return out;
}

} // namespace lvio_fusion
